#requires -Version 7.0
<#
本地、只读采集 Codex JSONL 的逐调用 usage；不连接机器人。
只导出用量、时间、模型与关联 ID，不导出消息、工具正文或推理内容。
统计结果是日志已知小计，不是账单；必须显式指定任务根会话。
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string[]]$ThreadId,
    [Parameter(Mandatory)][string]$OutputDirectory,
    [string]$WorkspacePath,
    [string]$SessionHome,
    [string]$StartTimeUtc,
    [string]$EndTimeUtc,
    [string]$ScopeLabel = '所选任务会话及其子代理'
)
$ErrorActionPreference = 'Stop'
if (-not $WorkspacePath) { $WorkspacePath = Split-Path -Parent $PSScriptRoot }
if (-not $SessionHome) { $SessionHome = $env:CODEX_HOME }
if (-not $SessionHome) { throw '需要 SessionHome 或现有 CODEX_HOME。' }
$workspaceFull = [IO.Path]::GetFullPath($WorkspacePath).TrimEnd('\','/')
$snapshot = [DateTimeOffset]::UtcNow
$start = if ($StartTimeUtc) { [DateTimeOffset]::Parse($StartTimeUtc) } else { [DateTimeOffset]::MinValue }
$end = if ($EndTimeUtc) { [DateTimeOffset]::Parse($EndTimeUtc) } else { $snapshot }
if ($end -gt $snapshot) { $end = $snapshot }
if ($start -ge $end) { throw '开始时间必须早于结束时间。' }
$fields = @('input_tokens','cached_input_tokens','cache_write_input_tokens','output_tokens','reasoning_output_tokens','total_tokens')
function Open-SharedReader([string]$Path) {
    [IO.StreamReader]::new([IO.File]::Open($Path, 'Open', 'Read', ([IO.FileShare]::ReadWrite -bor [IO.FileShare]::Delete)))
}
function Read-RecordTime($Value) {
    # ConvertFrom-Json 在不同 PowerShell 7 版本可能返回字符串或 UTC DateTime。
    # DateTime 不能先隐式转成本地无时区字符串再 Parse，否则时间窗口会偏移。
    if ($Value -is [DateTimeOffset]) { return $Value }
    if ($Value -is [DateTime]) { return [DateTimeOffset]::new($Value) }
    [DateTimeOffset]::Parse([string]$Value, [Globalization.CultureInfo]::InvariantCulture)
}
function Sum-Usage($Rows) {
    $result = [ordered]@{}
    foreach ($field in $fields) {
        $value = 0L; $missing = 0
        foreach ($row in $Rows) {
            if ($null -eq $row.usage[$field]) { $missing++ } else { $value += [long]$row.usage[$field] }
        }
        $result[$field] = if ($missing -or $Rows.Count -eq 0) { $null } else { $value }
    }
    $result
}
function Format-Count($Value) {
    if ($null -eq $Value) { return '未知' }
    ([long]$Value).ToString('N0')
}
$inventory = [Collections.Generic.List[object]]::new()
$warnings = [Collections.Generic.List[string]]::new()
foreach ($folder in @('sessions','archived_sessions')) {
    $folderPath = Join-Path $SessionHome $folder
    if (-not (Test-Path -LiteralPath $folderPath -PathType Container)) { continue }
    foreach ($file in Get-ChildItem -LiteralPath $folderPath -Filter '*.jsonl' -Recurse -File) {
        $reader = $null
        try {
            $reader = Open-SharedReader $file.FullName
            $first = $reader.ReadLine() | ConvertFrom-Json -AsHashtable
            $m = $first.payload
            if ($first.type -ne 'session_meta' -or -not $m.cwd) { continue }
            if ([IO.Path]::GetFullPath($m.cwd).TrimEnd('\','/') -ine $workspaceFull) { continue }
            $agentPath = $m.agent_path
            $role = if ($m.source -is [System.Collections.IDictionary] -and $m.source.subagent) { 'subagent' } else { 'root' }
            if ($m.source.subagent.other) { $role = 'internal_' + [string]$m.source.subagent.other }
            $inventory.Add([ordered]@{
                id=$m.id; parent_thread_id=$m.parent_thread_id; forked_from_id=$m.forked_from_id
                created_at=$m.timestamp; agent_path=$agentPath; role=$role; path=$file.FullName
            })
        } catch { $warnings.Add('元数据未读到：' + $file.FullName + '；' + $_.Exception.GetType().Name) }
        finally { if ($reader) { $reader.Dispose() } }
    }
}
$selected = [Collections.Generic.HashSet[string]]::new()
foreach ($id in $ThreadId) {
    if (-not ($inventory | Where-Object id -eq $id)) { throw ('当前工作区未找到指定会话：' + $id) }
    [void]$selected.Add($id)
}
do {
    $changed = $false
    foreach ($m in $inventory) {
        if ($m.parent_thread_id -and $selected.Contains($m.parent_thread_id)) {
            if ($selected.Add($m.id)) { $changed = $true }
        }
    }
} while ($changed)
$calls = [Collections.Generic.List[object]]::new()
$seen = @{}
$sources = [Collections.Generic.List[object]]::new()
$duplicates = 0
foreach ($m in ($inventory | Where-Object { $selected.Contains($_.id) } | Sort-Object created_at,id)) {
    $reader = Open-SharedReader $m.path
    $lineNumber = 0; $rawCount = 0; $excludedInherited = 0; $lastEvent = $null; $lastThreadCounter = $null
    $firstUsageAt = $null; $lastUsageAt = $null; $lastLineAt = $null; $parseErrors = 0
    $models = @{}; $model = $null
    try {
        while ($null -ne ($line = $reader.ReadLine())) {
            $lineNumber++
            try { $r = $line | ConvertFrom-Json -AsHashtable -ErrorAction Stop }
            catch { $parseErrors++; continue }
            if (-not $r.timestamp) { continue }
            $time = Read-RecordTime $r.timestamp
            if ($time -gt $snapshot) { continue }
            $lastLineAt = $r.timestamp
            $p = $r.payload
            if ($r.type -eq 'turn_context') {
                $model = $p.model
                if ($p.turn_id) { $models[$p.turn_id] = $model }
                continue
            }
            if ($r.type -eq 'event_msg' -and $p.type -eq 'token_count') {
                $lastEvent = $p.info.total_token_usage
                continue
            }
            if ($r.type -ne 'token_usage_record') { continue }
            $rawCount++
            # 排除物理复制到子会话中的父会话记录；上下文新调用仍正常计数。
            if ($p.thread_id -ne $m.id) { $excludedInherited++; continue }
            $lastThreadCounter = $p.thread_token_usage
            if ($time -lt $start -or $time -gt $end) { continue }
            if (-not $p.response_id) { $warnings.Add('无调用ID，未纳入：' + $m.id + ':' + $lineNumber); continue }
            $usage = [ordered]@{}
            foreach ($field in $fields) {
                $usage[$field] = if ($p.usage.Contains($field) -and $null -ne $p.usage[$field]) { [long]$p.usage[$field] } else { $null }
                if ($null -ne $usage[$field] -and $usage[$field] -lt 0) { throw '发现负用量。' }
            }
            if ($null -eq $usage.input_tokens -or $null -eq $usage.output_tokens -or $null -eq $usage.total_tokens) {
                throw ('调用核心usage字段缺失：' + $p.response_id)
            }
            if ($usage.total_tokens -ne $usage.input_tokens + $usage.output_tokens) { throw '用量公式不一致。' }
            if ($usage.cached_input_tokens -gt $usage.input_tokens -or $usage.reasoning_output_tokens -gt $usage.output_tokens) { throw '子集用量超过父项。' }
            $fingerprint = $usage | ConvertTo-Json -Compress
            if ($seen.ContainsKey($p.response_id)) {
                if ($seen[$p.response_id] -ne $fingerprint) { throw ('重复调用用量冲突：' + $p.response_id) }
                $duplicates++; continue
            }
            $seen[$p.response_id] = $fingerprint
            if (-not $firstUsageAt) { $firstUsageAt = $r.timestamp }; $lastUsageAt = $r.timestamp
            $calls.Add([ordered]@{
                timestamp=$r.timestamp; thread_id=$p.thread_id; turn_id=$p.turn_id; root_turn_id=$p.root_turn_id
                response_id=$p.response_id; model=$(if ($models.ContainsKey([string]$p.turn_id)) {$models[$p.turn_id]} else {$model})
                usage=$usage; source_line=$lineNumber
            })
        }
    } finally { $reader.Dispose() }
    $sources.Add([ordered]@{
        id=$m.id; parent_thread_id=$m.parent_thread_id; forked_from_id=$m.forked_from_id; role=$m.role
        agent_path=$m.agent_path; source_file=$m.path; source_bytes_observed=(Get-Item -LiteralPath $m.path).Length
        raw_usage_records=$rawCount; excluded_inherited_records=$excludedInherited; unreadable_lines=$parseErrors
        first_included_usage_at=$firstUsageAt; last_included_usage_at=$lastUsageAt; last_observed_line_at=$lastLineAt
        event_counter_for_comparison_only=$lastEvent; thread_counter_for_comparison_only=$lastThreadCounter
    })
}
$byThread = @($sources | ForEach-Object {
    $source = $_; $rows = @($calls | Where-Object thread_id -eq $source.id)
    [ordered]@{thread_id=$source.id;role=$source.role;agent_path=$source.agent_path;call_count=$rows.Count
        usage=$(if ($rows.Count) { Sum-Usage $rows } else { $null }); status=$(if($rows.Count){'known_logged_usage'}else{'no_usage_record_not_zero'})}
})
$byTurn = @($calls | Group-Object -Property { $_.thread_id + '|' + $_.turn_id } | ForEach-Object {
    $rows = @($_.Group); [ordered]@{thread_id=$rows[0].thread_id;turn_id=$rows[0].turn_id;call_count=$rows.Count
        first_at=$rows[0].timestamp;last_at=$rows[-1].timestamp;usage=(Sum-Usage $rows)}
})
$total = Sum-Usage $calls
$report = [ordered]@{
    schema_version='1.1'; scope_label=$ScopeLabel; snapshot_at_utc=$snapshot.ToString('o'); snapshot_at_local=$snapshot.ToOffset([TimeSpan]::FromHours(8)).ToString('o')
    requested_root_threads=$ThreadId; workspace=$workspaceFull
    usage_time_range_utc=@{start=$(if($StartTimeUtc){$start.ToString('o')}else{$null});end=$end.ToString('o')}
    completeness='已知日志小计；进行中的调用、当前最终回复、无usage记录的内部代理及未提供的其他会话可能未覆盖'
    method='仅按token_usage_record.payload.usage逐调用求和；response_id全局去重；不叠加累计计数或父会话继承历史'
    call_count=$calls.Count; duplicate_response_records_removed=$duplicates; known_usage=$total
    fields_note='total=input+output；cached_input和cache_write_input包含在input中；reasoning_output包含在output中；未知字段为null；不是账户账单'
    sources=$sources; by_thread=$byThread; by_turn=$byTurn; warnings=@($warnings); calls=$calls
    official_field_reference='https://developers.openai.com/api/docs/guides/agents-api/observability'
}
[void][IO.Directory]::CreateDirectory([IO.Path]::GetFullPath($OutputDirectory))
$stem = 'token_usage_' + $snapshot.ToString('yyyyMMdd_HHmmss_fff') + 'Z'
$jsonPath = Join-Path $OutputDirectory ($stem + '.json')
$mdPath = Join-Path $OutputDirectory ($stem + '.md')
[IO.File]::WriteAllText($jsonPath, ($report | ConvertTo-Json -Depth 14), [Text.UTF8Encoding]::new($false))
$md = [Collections.Generic.List[string]]::new()
$md.Add('# 任务 token 用量快照'); $md.Add(''); $md.Add('范围：' + $ScopeLabel)
$md.Add(''); $md.Add('采样时间（北京时间）：' + $report.snapshot_at_local)
$md.Add(''); $md.Add('用量记录截止（UTC）：' + $end.ToString('o') + '；开始：' + $(if($StartTimeUtc){$start.ToString('o')}else{'所选日志最早可见记录'}))
$md.Add(''); $md.Add('已知小计：**' + (Format-Count $total.total_tokens) + ' tokens**，来自 ' + $calls.Count + ' 次唯一调用。')
$md.Add(''); $md.Add('| 输入 | 其中缓存输入 | 输出 | 其中推理输出 | 总量 |'); $md.Add('|---:|---:|---:|---:|---:|')
$md.Add(('| {0} | {1} | {2} | {3} | {4} |' -f (Format-Count $total.input_tokens),(Format-Count $total.cached_input_tokens),(Format-Count $total.output_tokens),(Format-Count $total.reasoning_output_tokens),(Format-Count $total.total_tokens)))
$md.Add(''); $md.Add('缓存输入已含在输入，推理输出已含在输出，不再重复相加。输入可重复包含历史、图像和工具结果；它不是用户文字数量或单次上下文长度。此快照不是费用账单。')
$md.Add(''); $md.Add('## 会话与代理'); $md.Add(''); $md.Add('| 会话 / 代理 | 角色 | 唯一调用 | 已知总量 |'); $md.Add('|---|---|---:|---:|')
foreach ($row in $byThread) {
    $countText = if ($row.usage) { $row.usage.total_tokens.ToString('N0') } else { '未知（无记录）' }
    $md.Add(('| {0}{1} | {2} | {3} | {4} |' -f $row.thread_id,$(if($row.agent_path){' / '+$row.agent_path}else{''}),$row.role,$row.call_count,$countText))
}
$md.Add(''); $md.Add('## 统计方法与缺口'); $md.Add(''); $md.Add($report.method + '。')
$md.Add(''); $md.Add($report.completeness + '。没有记录不能按零消耗解释。')
$md.Add(''); $md.Add('JSON保留逐调用用量、时间、响应/会话ID、来源行号，以及累计计数供对照；没有复制消息、工具正文或推理内容。累计event计数与逐调用累计可能不同，本报告统一采用逐调用字段，不混算。')
$md.Add(''); $md.Add('通用字段解释参考 [OpenAI 官方用量说明](https://developers.openai.com/api/docs/guides/agents-api/observability)；本地Codex字段和去重依据来自实际JSONL，不能据此断言各运行环境格式相同。')
foreach ($warning in $warnings) { $md.Add(''); $md.Add('- ' + $warning) }
$md.Add(''); $md.Add('对应数据：[' + [IO.Path]::GetFileName($jsonPath) + '](' + [IO.Path]::GetFileName($jsonPath) + ')')
[IO.File]::WriteAllText($mdPath, ($md -join "`n") + "`n", [Text.UTF8Encoding]::new($false))
[ordered]@{markdown=$mdPath;json=$jsonPath;snapshot_at_local=$report.snapshot_at_local;call_count=$calls.Count;known_usage=$total} | ConvertTo-Json -Depth 4

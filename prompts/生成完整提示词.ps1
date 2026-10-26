[CmdletBinding()]
param(
    [string]$LibraryRoot = ''
)

# 仅汇总本地 Markdown 和资料清单；不连接网络，不执行任何机器人配置。
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$utf8Strict = New-Object System.Text.UTF8Encoding($false, $true)
$utf8Plain = New-Object System.Text.UTF8Encoding($false)
# Windows PowerShell 5.1 通过 -File 启动时，参数默认值求值阶段的
# $PSScriptRoot 可能为空；在脚本主体内再解析脚本目录。
if ([string]::IsNullOrWhiteSpace($LibraryRoot)) {
    $LibraryRoot = $PSScriptRoot
    if ([string]::IsNullOrWhiteSpace($LibraryRoot)) {
        $LibraryRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
    }
}
$root = (Resolve-Path -LiteralPath $LibraryRoot).Path
$entries = @(
    [pscustomobject]@{ Path = '00_任务入口提示词.md'; Anchor = 'source-00'; Title = '任务入口提示词' },
    [pscustomobject]@{ Path = '01_本机硬件与控制接口.md'; Anchor = 'source-01'; Title = '本机硬件与控制接口' },
    [pscustomobject]@{ Path = '02_机器人与夹爪几何模型及标定.md'; Anchor = 'source-02'; Title = '机器人与夹爪几何模型及标定' },
    [pscustomobject]@{ Path = '03_执行流程与异常恢复.md'; Anchor = 'source-03'; Title = '执行流程与异常恢复' },
    [pscustomobject]@{ Path = '04_任务模板与成功判据.md'; Anchor = 'source-04'; Title = '任务模板与成功判据' },
    [pscustomobject]@{ Path = '模型与配置/README.md'; Anchor = 'source-model-config'; Title = '模型与配置说明' },
    [pscustomobject]@{ Path = '模型与配置/本机配置与验证状态.json'; Anchor = 'source-machine-config'; Title = '本机配置与验证状态清单' }
)

function Read-StrictUtf8([string]$Path) {
    $text = $utf8Strict.GetString([System.IO.File]::ReadAllBytes($Path))
    if ($text.Length -gt 0 -and $text[0] -eq [char]0xfeff) { $text = $text.Substring(1) }
    return $text
}

function Get-RelativeLibraryPath([string]$AbsolutePath) {
    $prefix = $root.TrimEnd([char[]]'\/') + [System.IO.Path]::DirectorySeparatorChar
    if (-not $AbsolutePath.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $null
    }
    return $AbsolutePath.Substring($prefix.Length).Replace('\', '/')
}

# 所有必要输入先核对和读取。缺失或无效时，不覆盖任何已有导出。
$missing = @($entries | Where-Object { -not (Test-Path -LiteralPath (Join-Path $root $_.Path) -PathType Leaf) })
if ($missing.Count -gt 0) {
    throw ('缺少维护源文件，未生成导出：' + (($missing | ForEach-Object { $_.Path }) -join '；'))
}
$contents = @{}
$anchorByPath = @{}
$sourceRecords = @()
foreach ($entry in $entries) {
    $path = Join-Path $root $entry.Path
    $sourceBytes = [System.IO.File]::ReadAllBytes($path)
    $contents[$entry.Path] = $utf8Strict.GetString($sourceBytes).TrimStart([char]0xfeff)
    $anchorByPath[$entry.Path.ToLowerInvariant()] = $entry.Anchor
    $sourceSha = [System.Security.Cryptography.SHA256]::Create()
    try { $sourceHash = ([BitConverter]::ToString($sourceSha.ComputeHash($sourceBytes))).Replace('-', '').ToLowerInvariant() }
    finally { $sourceSha.Dispose() }
    $sourceRecords += [pscustomobject][ordered]@{
        relativePath = $entry.Path
        sha256 = $sourceHash
        sizeBytes = $sourceBytes.Length
    }
}
$config = $contents['模型与配置/本机配置与验证状态.json'] | ConvertFrom-Json -ErrorAction Stop
if ($null -eq $config -or $config -is [System.Array] -or $config -is [string] -or $config -is [ValueType] -or @($config.PSObject.Properties).Count -eq 0) {
    throw '本机配置与验证状态.json 必须是具有属性的 JSON 对象；未生成导出。'
}
$versionProperty = $config.PSObject.Properties['library_version']
if ($null -eq $versionProperty -or [string]::IsNullOrWhiteSpace([string]$versionProperty.Value)) {
    throw '本机配置与验证状态.json 缺少非空的顶层 library_version；未生成导出。'
}
$libraryVersion = [string]$versionProperty.Value

function Convert-SourceMarkdown([string]$Markdown, [string]$SourcePath, [string]$SourceAnchor) {
    $sourceDir = Split-Path -Parent (Join-Path $root $SourcePath)
    $lines = $Markdown -split '\r?\n'
    $out = New-Object System.Collections.Generic.List[string]
    $inFence = $false
    $fenceChar = ''
    $fenceLength = 0
    foreach ($line in $lines) {
        $fence = [regex]::Match($line, '^\s{0,3}(`{3,}|~{3,})')
        if ($fence.Success) {
            $mark = $fence.Groups[1].Value
            if (-not $inFence) {
                $inFence = $true; $fenceChar = $mark.Substring(0, 1); $fenceLength = $mark.Length
            } elseif ($mark.Substring(0, 1) -eq $fenceChar -and $mark.Length -ge $fenceLength -and $line.Trim() -eq $mark) {
                $inFence = $false
            }
            $out.Add($line)
            continue
        }
        if ($inFence) { $out.Add($line); continue }

        # 新建来源段落后，把来源自身的一级标题降为二级；其他层级保留。
        $converted = [regex]::Replace($line, '^#\s+', '## ')
        $evaluator = [System.Text.RegularExpressions.MatchEvaluator]{
            param($match)
            $label = $match.Groups['label'].Value
            $target = $match.Groups['target'].Value.Trim()
            if ($target.StartsWith('<') -and $target.EndsWith('>')) { $target = $target.Substring(1, $target.Length - 2) }
            # 本库的链接使用简单路径；外部网址作为可选出处保留。
            if ($target -match '^(https?://|mailto:)') { return $match.Value }
            if ($target.StartsWith('#')) { return ('[' + $label + '](#' + $SourceAnchor + ')') }
            $linkPath = ($target -split '#', 2)[0]
            $linkPath = ($linkPath -split '\?', 2)[0]
            try { $linkPath = [System.Uri]::UnescapeDataString($linkPath) } catch { }
            try {
                $absolute = [System.IO.Path]::GetFullPath((Join-Path $sourceDir $linkPath))
                $relative = Get-RelativeLibraryPath $absolute
            } catch { $relative = $null }
            if ($null -ne $relative -and $anchorByPath.ContainsKey($relative.ToLowerInvariant())) {
                return ('[' + $label + '](#' + $anchorByPath[$relative.ToLowerInvariant()] + ')')
            }
            if ($null -ne $relative -and $relative -eq 'README.md') {
                return ('[' + $label + '](#export-usage)')
            }
            # 不把维护脚本、旧实验或原目录链接变成单文件的必要依赖。
            return ($label + '（原库可选资源：' + $target + '；本文件不自动执行或读取）')
        }
        $converted = [regex]::Replace($converted, '!?\[(?<label>[^\]\r\n]+)\]\((?<target><[^>\r\n]+>|[^)\r\n]+)\)', $evaluator)
        $out.Add($converted)
    }
    if ($inFence) { throw ('来源 Markdown 代码围栏未闭合：' + $SourcePath) }
    return ($out -join "`n")
}

$builtAtUtc = [DateTimeOffset]::UtcNow.ToString('o')
$parts = New-Object System.Collections.Generic.List[string]
$parts.Add('# 宇树 G1 机器人完整任务提示词')
$parts.Add('')
$parts.Add('资料库版本：' + $libraryVersion + '；生成时间（UTC）：' + $builtAtUtc + '。')
$parts.Add('')
$parts.Add('<a id="export-usage"></a>')
$parts.Add('## 使用说明')
$parts.Add('')
$parts.Add('这是一份独立阅读版本，已包含任务入口、硬件接口、几何与标定、执行恢复、任务模板、模型说明和配置状态清单。可以整份交给新的 GPT，无须先找到其他旧 Markdown 或历史实验复盘。必需的执行要求已在本文展开。')
$parts.Add('')
$parts.Add('本文中的清单与历史核实记录不代表机器人当前状态，也不授予任何实际运动。SSH、SDK、模型文件、串口权限、现场条件和实时反馈等实际能力仍须按本文核对；不能用文字代替缺失模型、尺寸、标定或安全交接。JSON 附录仅供查阅，不是可直接加载的执行配置。')
$parts.Add('')
$parts.Add('目录中的链接指向本文件内部来源段落。请填写本文「任务模板与成功判据」中的本次任务模板，按任务所需部分完成准备与验证。原始脚本、证据和可选历史资源仍在维护库及工作区中；它们不构成本文件的阅读前提。')
$parts.Add('')
foreach ($entry in $entries) { $parts.Add('- [' + $entry.Title + '](#' + $entry.Anchor + ')') }
foreach ($entry in $entries) {
    $parts.Add('')
    $parts.Add('---')
    $parts.Add('')
    $parts.Add('<a id="' + $entry.Anchor + '"></a>')
    $parts.Add('## 来源段落：' + $entry.Title)
    $parts.Add('')
    if ($entry.Path.EndsWith('.json')) {
        $parts.Add('下列内容是设备、模型与验证状态的资料清单；不自动执行、不代替现场检查。')
        $parts.Add('')
        $parts.Add('```json')
        $parts.Add($contents[$entry.Path].TrimEnd())
        $parts.Add('```')
    } else {
        $parts.Add((Convert-SourceMarkdown $contents[$entry.Path] $entry.Path $entry.Anchor).TrimEnd())
    }
}
$outputText = ($parts -join "`n") + "`n"
$outputBytes = $utf8Plain.GetBytes($outputText)
$sha = [System.Security.Cryptography.SHA256]::Create()
try { $outputHash = ([BitConverter]::ToString($sha.ComputeHash($outputBytes))).Replace('-', '').ToLowerInvariant() }
finally { $sha.Dispose() }
$manifest = [ordered]@{
    schemaVersion = 1
    libraryVersion = $libraryVersion
    builtAtUtc = $builtAtUtc
    generator = '生成完整提示词.ps1'
    sources = @($sourceRecords)
    export = [ordered]@{
        relativePath = '导出/G1机器人完整提示词.md'
        sha256 = $outputHash
        sizeBytes = $outputBytes.Length
    }
}
$manifestText = ($manifest | ConvertTo-Json -Depth 12) + "`n"
$exportDir = Join-Path $root '导出'
[System.IO.Directory]::CreateDirectory($exportDir) | Out-Null
$token = [Guid]::NewGuid().ToString('N')
$tempOutput = Join-Path $exportDir ('.完整提示词-' + $token + '.tmp')
$tempManifest = Join-Path $exportDir ('.生成清单-' + $token + '.tmp')
try {
    [System.IO.File]::WriteAllBytes($tempOutput, $outputBytes)
    [System.IO.File]::WriteAllText($tempManifest, $manifestText, $utf8Plain)
    Move-Item -LiteralPath $tempOutput -Destination (Join-Path $exportDir 'G1机器人完整提示词.md') -Force
    Move-Item -LiteralPath $tempManifest -Destination (Join-Path $exportDir '生成清单.json') -Force
} finally {
    if (Test-Path -LiteralPath $tempOutput) { Remove-Item -LiteralPath $tempOutput -Force }
    if (Test-Path -LiteralPath $tempManifest) { Remove-Item -LiteralPath $tempManifest -Force }
}
Write-Output ('已生成：' + (Join-Path $exportDir 'G1机器人完整提示词.md'))
Write-Output ('来源文件：' + $entries.Count + '；SHA-256：' + $outputHash)

[CmdletBinding()]
param(
    [string]$LibraryRoot = '',
    [switch]$SkipExport
)

# 只读校验资料文件；不连接网络，不调用机器人、不更改导出。
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$utf8Strict = New-Object System.Text.UTF8Encoding($false, $true)
# 避免 Windows PowerShell 5.1 的 -File 参数默认值早于脚本目录初始化。
if ([string]::IsNullOrWhiteSpace($LibraryRoot)) {
    $LibraryRoot = $PSScriptRoot
    if ([string]::IsNullOrWhiteSpace($LibraryRoot)) {
        $LibraryRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
    }
}
$root = (Resolve-Path -LiteralPath $LibraryRoot).Path
$issues = New-Object System.Collections.Generic.List[string]
$required = @(
    'README.md',
    '00_任务入口提示词.md',
    '01_本机硬件与控制接口.md',
    '02_机器人与夹爪几何模型及标定.md',
    '03_执行流程与异常恢复.md',
    '04_任务模板与成功判据.md',
    '维护记录.md',
    '生成完整提示词.ps1',
    '校验资料库.ps1',
    '模型与配置/README.md',
    '模型与配置/本机配置与验证状态.json'
)
$sourcePaths = @(
    '00_任务入口提示词.md',
    '01_本机硬件与控制接口.md',
    '02_机器人与夹爪几何模型及标定.md',
    '03_执行流程与异常恢复.md',
    '04_任务模板与成功判据.md',
    '模型与配置/README.md',
    '模型与配置/本机配置与验证状态.json'
)
if (-not $SkipExport) { $required += @('导出/G1机器人完整提示词.md', '导出/生成清单.json') }

function Add-Issue([string]$Message) { $issues.Add($Message) }

function Read-StrictUtf8([string]$Path) {
    $value = $utf8Strict.GetString([System.IO.File]::ReadAllBytes($Path))
    if ($value.Length -gt 0 -and $value[0] -eq [char]0xfeff) { $value = $value.Substring(1) }
    return $value
}

function Get-Sha256Hex([string]$Path) {
    # 直接使用 .NET，避免独立子进程的 PSModulePath/模块自动加载差异。
    $sha = [System.Security.Cryptography.SHA256]::Create()
    try {
        $bytes = [System.IO.File]::ReadAllBytes($Path)
        return ([BitConverter]::ToString($sha.ComputeHash($bytes))).Replace('-', '').ToLowerInvariant()
    } finally { $sha.Dispose() }
}

function Get-PropertyValue($Object, [string]$Name) {
    if ($null -eq $Object) { return $null }
    $property = $Object.PSObject.Properties[$Name]
    if ($null -eq $property) { return $null }
    return $property.Value
}

function Test-Markdown([string]$RelativePath, [string]$Text) {
    $inFence = $false
    $fenceChar = ''
    $fenceLength = 0
    $fenceLine = 0
    $headingCount = 0
    $headingIds = New-Object System.Collections.Generic.List[string]
    $links = New-Object System.Collections.Generic.List[object]
    $lineNumber = 0
    foreach ($line in ($Text -split '\r?\n')) {
        $lineNumber++
        $fence = [regex]::Match($line, '^\s{0,3}(`{3,}|~{3,})')
        if ($fence.Success) {
            $mark = $fence.Groups[1].Value
            if (-not $inFence) {
                $inFence = $true; $fenceChar = $mark.Substring(0, 1); $fenceLength = $mark.Length; $fenceLine = $lineNumber
            } elseif ($mark.Substring(0, 1) -eq $fenceChar -and $mark.Length -ge $fenceLength -and $line.Trim() -eq $mark) {
                $inFence = $false
            }
            continue
        }
        if ($inFence) { continue }
        $heading = [regex]::Match($line, '^\s{0,3}(#{1,6})\s+(.+?)\s*#*\s*$')
        if ($heading.Success) {
            $headingCount++
            # 与常用 Markdown 渲染器相近；中文、ASCII 字母数字及连字符保留。
            $id = $heading.Groups[2].Value.ToLowerInvariant()
            $id = [regex]::Replace($id, '[^\p{L}\p{Nd}_\-\s]', '')
            $id = [regex]::Replace($id.Trim(), '\s', '-')
            $headingIds.Add($id)
        } elseif ($line -match '^\s{0,3}#{7,}\s') {
            Add-Issue ($RelativePath + ':' + $lineNumber + ' 标题超过六级。')
        }
        foreach ($anchor in [regex]::Matches($line, '<a\s+(?:id|name)=["'']([^"'']+)["'']\s*></a>')) { $headingIds.Add($anchor.Groups[1].Value) }
        foreach ($link in [regex]::Matches($line, '!?\[[^\]\r\n]+\]\((?<target><[^>\r\n]+>|[^)\r\n]+)\)')) {
            $links.Add([pscustomobject]@{ Target = $link.Groups['target'].Value.Trim(); Line = $lineNumber })
        }
    }
    if ($inFence) { Add-Issue ($RelativePath + ':' + $fenceLine + ' 代码围栏未闭合。') }
    if ($headingCount -eq 0) { Add-Issue ($RelativePath + ' 未发现 Markdown 标题。') }
    foreach ($link in $links) {
        $target = $link.Target
        if ($target.StartsWith('<') -and $target.EndsWith('>')) { $target = $target.Substring(1, $target.Length - 2) }
        if ($target -match '^[A-Za-z][A-Za-z0-9+.-]*:') {
            # 网络来源不做联网校验；本脚本不能确认其有效性或权限。
            continue
        }
        if ($target.StartsWith('#')) {
            $fragment = [System.Uri]::UnescapeDataString($target.Substring(1))
            if ($fragment.Length -gt 0 -and -not $headingIds.Contains($fragment)) { Add-Issue ($RelativePath + ':' + $link.Line + ' 内部锚点不存在：' + $target) }
            continue
        }
        $pathPart = ($target -split '#', 2)[0]
        $pathPart = ($pathPart -split '\?', 2)[0]
        try {
            $pathPart = [System.Uri]::UnescapeDataString($pathPart)
            $fullPath = [System.IO.Path]::GetFullPath((Join-Path (Split-Path -Parent (Join-Path $root $RelativePath)) $pathPart))
            $expectedExport = [System.IO.Path]::GetFullPath((Join-Path $root '导出/G1机器人完整提示词.md'))
            if ($SkipExport -and $fullPath -eq $expectedExport) { continue }
            if (-not (Test-Path -LiteralPath $fullPath)) {
                Add-Issue ($RelativePath + ':' + $link.Line + ' 相对链接目标不存在：' + $target)
            }
            if ($RelativePath -eq '导出/G1机器人完整提示词.md') {
                Add-Issue ($RelativePath + ':' + $link.Line + ' 单文件导出保留了外部文件链接：' + $target)
            }
        } catch { Add-Issue ($RelativePath + ':' + $link.Line + ' 链接路径无效：' + $target) }
    }
}

$texts = @{}
foreach ($relative in $required) {
    $path = Join-Path $root $relative
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { Add-Issue ('缺少必要文件：' + $relative); continue }
    try {
        $texts[$relative] = Read-StrictUtf8 $path
        if ($texts[$relative].Trim().Length -eq 0) { Add-Issue ('文件为空：' + $relative) }
    } catch { Add-Issue ('无效 UTF-8 或无法读取：' + $relative + '；' + $_.Exception.Message); continue }
    if ($relative.EndsWith('.md')) { Test-Markdown $relative $texts[$relative] }
    if ($relative.EndsWith('.json')) {
        try {
            $json = $texts[$relative] | ConvertFrom-Json -ErrorAction Stop
            if ($null -eq $json -or $json -is [System.Array] -or $json -is [string] -or $json -is [ValueType] -or @($json.PSObject.Properties).Count -eq 0) {
                Add-Issue ('JSON 必须为具有属性的对象：' + $relative)
            }
            if ($relative -eq '模型与配置/本机配置与验证状态.json' -and [string]::IsNullOrWhiteSpace([string](Get-PropertyValue $json 'library_version'))) {
                Add-Issue '本机配置与验证状态.json 缺少非空的顶层 library_version。'
            }
        } catch { Add-Issue ('JSON 无法解析：' + $relative + '；' + $_.Exception.Message) }
    }
    if ($relative.EndsWith('.ps1')) {
        $tokens = $null; $parseErrors = $null
        [System.Management.Automation.Language.Parser]::ParseInput($texts[$relative], [ref]$tokens, [ref]$parseErrors) | Out-Null
        foreach ($parseError in @($parseErrors)) { Add-Issue ($relative + ' PowerShell 语法错误：' + $parseError.Message) }
    }
}

if (-not $SkipExport -and $texts.ContainsKey('导出/生成清单.json')) {
    try {
        $manifest = $texts['导出/生成清单.json'] | ConvertFrom-Json -ErrorAction Stop
        if ((Get-PropertyValue $manifest 'schemaVersion') -ne 1) { Add-Issue '生成清单 schemaVersion 应为 1。' }
        if ([string]::IsNullOrWhiteSpace([string](Get-PropertyValue $manifest 'libraryVersion'))) { Add-Issue '生成清单缺少 libraryVersion。' }
        if ($texts.ContainsKey('模型与配置/本机配置与验证状态.json')) {
            $machineConfig = $texts['模型与配置/本机配置与验证状态.json'] | ConvertFrom-Json -ErrorAction Stop
            $expectedVersion = [string](Get-PropertyValue $machineConfig 'library_version')
            if ([string]::IsNullOrWhiteSpace($expectedVersion)) {
                Add-Issue '本机配置与验证状态.json 缺少非空的顶层 library_version。'
            } elseif ([string](Get-PropertyValue $manifest 'libraryVersion') -ne $expectedVersion) {
                Add-Issue '生成清单版本与配置清单 library_version 不一致；请重新生成。'
            }
        }
        $buildTime = [DateTimeOffset]::MinValue
        if (-not [DateTimeOffset]::TryParse([string](Get-PropertyValue $manifest 'builtAtUtc'), [ref]$buildTime)) { Add-Issue '生成清单缺少有效构建时间。' }
        $records = @(Get-PropertyValue $manifest 'sources')
        $seen = @{}
        foreach ($record in $records) {
            $relative = [string](Get-PropertyValue $record 'relativePath')
            if ($sourcePaths -notcontains $relative) { Add-Issue ('生成清单含未知来源：' + $relative); continue }
            if ($seen.ContainsKey($relative)) { Add-Issue ('生成清单来源重复：' + $relative); continue }
            $seen[$relative] = $true
            $path = Join-Path $root $relative
            if (Test-Path -LiteralPath $path -PathType Leaf) {
                $actualHash = Get-Sha256Hex $path
                if ($actualHash -ne [string](Get-PropertyValue $record 'sha256')) { Add-Issue ('源文件已更新，导出过期：' + $relative) }
                if ((Get-Item -LiteralPath $path).Length -ne (Get-PropertyValue $record 'sizeBytes')) { Add-Issue ('来源文件大小不匹配：' + $relative) }
            }
        }
        foreach ($relative in $sourcePaths) { if (-not $seen.ContainsKey($relative)) { Add-Issue ('生成清单缺少来源：' + $relative) } }
        $export = Get-PropertyValue $manifest 'export'
        if ((Get-PropertyValue $export 'relativePath') -ne '导出/G1机器人完整提示词.md') { Add-Issue '生成清单的导出路径不符合约定。' }
        $exportPath = Join-Path $root '导出/G1机器人完整提示词.md'
        if (Test-Path -LiteralPath $exportPath -PathType Leaf) {
            $actualHash = Get-Sha256Hex $exportPath
            if ($actualHash -ne [string](Get-PropertyValue $export 'sha256')) { Add-Issue '导出 SHA-256 不匹配；请重新生成。' }
            if ((Get-Item -LiteralPath $exportPath).Length -ne (Get-PropertyValue $export 'sizeBytes')) { Add-Issue '导出文件大小不匹配。' }
            $exportText = $texts['导出/G1机器人完整提示词.md']
            foreach ($anchor in @('source-00', 'source-01', 'source-02', 'source-03', 'source-04', 'source-model-config', 'source-machine-config')) {
                if (-not $exportText.Contains('<a id="' + $anchor + '"></a>')) { Add-Issue ('导出缺少来源段落：' + $anchor) }
            }
        }
    } catch { Add-Issue ('生成清单核对失败：' + $_.Exception.Message) }
}

if ($issues.Count -gt 0) {
    foreach ($issue in $issues) { Write-Output ('失败：' + $issue) }
    throw ('资料库校验未通过，共 ' + $issues.Count + ' 项。')
}
if ($SkipExport) {
    Write-Output ('源资料校验通过：' + $required.Count + ' 个必要文件；本次未检查导出。')
} else {
    Write-Output ('资料库校验通过：' + $required.Count + ' 个必要文件；UTF-8、JSON、标题与围栏、相对链接及来源/导出 SHA-256 一致。')
}
Write-Output '本校验只确认文档与构建一致性，不代表模型、标定、实时状态或机器人运动已验证。'

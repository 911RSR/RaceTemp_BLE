$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$appConfPath = Join-Path $repoRoot 'Core\Inc\app_conf.h'
$customStmPath = Join-Path $repoRoot 'STM32_WPAN\App\custom_stm.c'
$customAppPath = Join-Path $repoRoot 'STM32_WPAN\App\custom_app.c'
$linkerScriptPath = Join-Path $repoRoot 'STM32WB55CGUX_FLASH.ld'

function Remove-SecondExactBlock {
    param(
        [Parameter(Mandatory = $true)] [string] $Text,
        [Parameter(Mandatory = $true)] [string] $Block
    )

    $first = $Text.IndexOf($Block)
    if ($first -lt 0) { return $Text }

    $second = $Text.IndexOf($Block, $first + $Block.Length)
    if ($second -lt 0) { return $Text }

    return $Text.Remove($second, $Block.Length)
}

function Remove-DuplicateFunction {
    param(
        [Parameter(Mandatory = $true)] [string] $Text,
        [Parameter(Mandatory = $true)] [string] $FunctionName
    )

    $pattern = "(?m)^(?:__USED\s+)?void\s+$([regex]::Escape($FunctionName))\s*\(void\)\s*/\* Property (?:Read|Notification) \*/\s*\r?\n\{"
    $matches = [regex]::Matches($Text, $pattern)
    if ($matches.Count -lt 2) { return $Text }

    $removeRanges = @()
    for ($matchIndex = 1; $matchIndex -lt $matches.Count; $matchIndex++) {
        $match = $matches[$matchIndex]
        $start = $match.Index
        $openBrace = $Text.IndexOf('{', $match.Index)
        $depth = 0
        $end = -1

        for ($i = $openBrace; $i -lt $Text.Length; $i++) {
            if ($Text[$i] -eq '{') {
                $depth++
            } elseif ($Text[$i] -eq '}') {
                $depth--
                if ($depth -eq 0) {
                    $end = $i + 1
                    break
                }
            }
        }

        if ($end -lt 0) {
            throw "Could not find the end of duplicate function $FunctionName"
        }

        $lineStart = $Text.LastIndexOf("`n", [Math]::Max(0, $start - 1))
        if ($lineStart -lt 0) { $lineStart = 0 } else { $lineStart++ }
        $previousLineEnd = $lineStart
        $previousLineStart = if ($previousLineEnd -gt 1) { $Text.LastIndexOf("`n", $previousLineEnd - 2) } else { -1 }
        if ($previousLineStart -lt 0) { $previousLineStart = 0 } else { $previousLineStart++ }
        $previousLine = $Text.Substring($previousLineStart, $previousLineEnd - $previousLineStart).Trim()
        if ($previousLine -match '^/\* Race(Temp|Chrono) \*/$') {
            $start = $previousLineStart
        }

        while (($end -lt $Text.Length) -and (($Text[$end] -eq "`r") -or ($Text[$end] -eq "`n"))) {
            $end++
        }

        $removeRanges += [PSCustomObject]@{ Start = $start; Length = $end - $start }
    }

    foreach ($range in ($removeRanges | Sort-Object Start -Descending)) {
        $Text = $Text.Remove($range.Start, $range.Length)
    }

    return $Text
}

$appConf = Get-Content -LiteralPath $appConfPath -Raw
$appConf = [regex]::Replace(
    $appConf,
    '#define\s+CFG_MITM_PROTECTION\s+CFG_MITM_PROTECTION_REQUIRED',
    '#define CFG_MITM_PROTECTION                   CFG_MITM_PROTECTION_NOT_REQUIRED'
)
$appConf = $appConf.Replace(
    'SHCI_C2_BLE_INIT_OPTIONS_GATT_CACHING_USED',
    'SHCI_C2_BLE_INIT_OPTIONS_GATT_CACHING_NOTUSED'
)
Set-Content -LiteralPath $appConfPath -Value $appConf -NoNewline

$customStm = Get-Content -LiteralPath $customStmPath -Raw
$customStm = [regex]::Replace(
    $customStm,
    '(?ms)(uint16_t\s+SizeCan_Main\s*=\s*\d+;\r?\nuint16_t\s+SizeCan_Filter\s*=\s*\d+;\r?\n)\1',
    '$1'
)
Set-Content -LiteralPath $customStmPath -Value $customStm -NoNewline

$customApp = Get-Content -LiteralPath $customAppPath -Raw
$customApp = [regex]::Replace(
    $customApp,
    '(?ms)(/\* Race(?:Temp|Chrono) \*/\r?\nstatic void Custom_Can_main_Update_Char\(void\);\r?\nstatic void Custom_Can_main_Send_Notification\(void\);\r?\n)\s*\1',
    '$1'
)
$customApp = Remove-DuplicateFunction -Text $customApp -FunctionName 'Custom_Can_main_Update_Char'
$customApp = Remove-DuplicateFunction -Text $customApp -FunctionName 'Custom_Can_main_Send_Notification'
$customApp = [regex]::Replace(
    $customApp,
    '(?:__USED\s+)*void Custom_Can_main_Send_Notification\(void\) /\* Property Notification \*/',
    '__USED void Custom_Can_main_Send_Notification(void) /* Property Notification */'
)
Set-Content -LiteralPath $customAppPath -Value $customApp -NoNewline

if (Test-Path -LiteralPath $linkerScriptPath) {
    $linkerScript = Get-Content -LiteralPath $linkerScriptPath -Raw
    $linkerScript = [regex]::Replace(
        $linkerScript,
        'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x08000000,\s*LENGTH\s*=\s*\d+K',
        'FLASH (rx)                 : ORIGIN = 0x08000000, LENGTH = 504K'
    )
    if ($linkerScript -notmatch 'Last two 4 KB flash pages are reserved') {
        $linkerScript = [regex]::Replace(
            $linkerScript,
            "(MEMORY\r?\n\{\r?\n)",
            "`$1/* Last two 4 KB flash pages are reserved for RaceTemp persistent counters. */`r`n",
            1
        )
    }
    if ($linkerScript -notmatch 'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x08000000,\s*LENGTH\s*=\s*504K') {
        throw 'Could not reserve RaceTemp counter flash pages in linker script.'
    }
    Set-Content -LiteralPath $linkerScriptPath -Value $linkerScript -NoNewline
}

Write-Host 'Fixed CubeMX BLE stubs, BLE settings, and RaceTemp flash reservation when present.'

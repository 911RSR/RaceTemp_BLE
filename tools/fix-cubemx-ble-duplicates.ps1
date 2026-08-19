$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$appConfPath = Join-Path $repoRoot 'Core\Inc\app_conf.h'
$mainPath = Join-Path $repoRoot 'Core\Src\main.c'
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

function Repair-LinkerScript {
    param(
        [Parameter(Mandatory = $true)] [string] $Text
    )

    $Text = [regex]::Replace(
        $Text,
        'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x08000000,\s*LENGTH\s*=\s*\d+K',
        'FLASH (rx)                 : ORIGIN = 0x08000000, LENGTH = 504K'
    )
    if ($Text -notmatch 'Last two 4 KB flash pages are reserved') {
        $Text = [regex]::Replace(
            $Text,
            "(MEMORY\r?\n\{\r?\n)",
            "`$1/* Last two 4 KB flash pages are reserved for RaceTemp persistent counters. */`r`n",
            1
        )
    }
    if ($Text -notmatch 'PERSIST\s+\(rx\)') {
        $Text = [regex]::Replace(
            $Text,
            '(FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x08000000,\s*LENGTH\s*=\s*504K\r?\n)',
            "`$1PERSIST (rx)               : ORIGIN = 0x0807E000, LENGTH = 8K`r`n",
            1
        )
    }
    $Text = [regex]::Replace(
        $Text,
        '(?m)^\s*RAM_SHARED\s+\(xrw\)\s*:\s*ORIGIN\s*=\s*0x20030000,\s*LENGTH\s*=\s*\d+K',
        'RAM_SHARED (xrw)           : ORIGIN = 0x20030000, LENGTH = 2K',
        1
    )
    $Text = [regex]::Replace(
        $Text,
        '/\*\s*RAM_SHARED\s+\(xrw\)\s*:\s*ORIGIN\s*=\s*0x20030000,\s*LENGTH\s*=\s*\d+K\s*\*/',
        '/* RAM_SHARED (xrw)           : ORIGIN = 0x20030000, LENGTH = 10K */',
        1
    )
    if ($Text -notmatch 'RAMB_SHARED\s+\(xrw\)') {
        $Text = [regex]::Replace(
            $Text,
            '(RAM_SHARED\s+\(xrw\)\s*:\s*ORIGIN\s*=\s*0x20030000,\s*LENGTH\s*=\s*2K\r?\n)',
            "`$1RAMB_SHARED (xrw)          : ORIGIN = 0x20038000, LENGTH = 10K`r`n/* RAM_SHARED (xrw)           : ORIGIN = 0x20030000, LENGTH = 10K */`r`n",
            1
        )
    }

    if ($Text -notmatch '\*\(BLE_DRIVER_CONTEXT\)') {
        $Text = [regex]::Replace(
            $Text,
            '(\s+\*\(\.RamFunc\*\)\s*/\* \.RamFunc\* sections \*/\r?\n)',
            "`$1    *(BLE_DRIVER_CONTEXT)`r`n",
            1
        )
    }
    if ($Text -notmatch '\*\(SYSTEM_DRIVER_CONTEXT\)') {
        $Text = [regex]::Replace(
            $Text,
            '(\s+\*\(BLE_DRIVER_CONTEXT\)\r?\n)',
            "`$1    *(SYSTEM_DRIVER_CONTEXT)`r`n",
            1
        )
    }

    $Text = [regex]::Replace(
        $Text,
        '\._user_heap_stack\s*(?:\(NOLOAD\)\s*)?:',
        '._user_heap_stack (NOLOAD) :',
        1
    )

    $Text = [regex]::Replace(
        $Text,
        'MB_MEM1\s+\(NOLOAD\)\s*:\s*\{\s*\*\(MB_MEM1\)\s*\}\s*>RAM_SHARED',
        'MB_MEM1 (NOLOAD)       : { *(MB_MEM1) } >RAMB_SHARED',
        1
    )

    $mbMem2Pattern = '(?ms)\s*/\*\s*used by the startup to initialize \.MB_MEM2 data\s*\*/\s*_siMB_MEM2\s*=\s*LOADADDR\(\.MB_MEM2\);\s*\.MB_MEM2\s*:\s*\{.*?\}\s*>RAM(?:B)?_SHARED\s*AT>\s*FLASH'
    $mbMem2Replacement = @'

  .MB_MEM2 ALIGN(4) : AT(ALIGN(LOADADDR(.data) + SIZEOF(.data), 4))
  {
    . = ALIGN(4);
    _sMB_MEM2 = . ;
    *(MB_MEM2) ;
    . = ALIGN(4);
    _eMB_MEM2 = . ;
  } >RAMB_SHARED

  /* used by the startup to initialize .MB_MEM2 data */
  _siMB_MEM2 = LOADADDR(.MB_MEM2);
'@
    $Text = [regex]::Replace($Text, $mbMem2Pattern, $mbMem2Replacement, 1)
    $Text = [regex]::Replace(
        $Text,
        '(?ms)(\.MB_MEM2\s+ALIGN\(4\)\s*:\s*AT\(ALIGN\(LOADADDR\(\.data\)\s*\+\s*SIZEOF\(\.data\),\s*4\)\)\s*\{.*?\}\s*)>RAM_SHARED',
        '$1>RAMB_SHARED',
        1
    )

    if ($Text -notmatch 'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x08000000,\s*LENGTH\s*=\s*504K') {
        throw 'Could not reserve RaceTemp counter flash pages in linker script.'
    }
    foreach ($required in @('PERSIST\s+\(rx\)', 'RAM_SHARED\s+\(xrw\)\s*:\s*ORIGIN\s*=\s*0x20030000,\s*LENGTH\s*=\s*2K', 'RAMB_SHARED\s+\(xrw\)\s*:\s*ORIGIN\s*=\s*0x20038000,\s*LENGTH\s*=\s*10K', '(?s)MB_MEM1\s+\(NOLOAD\).*?>RAMB_SHARED', '\*\(BLE_DRIVER_CONTEXT\)', '\*\(SYSTEM_DRIVER_CONTEXT\)', '\._user_heap_stack\s*\(NOLOAD\)', '(?s)\.MB_MEM2\s+ALIGN\(4\).*?>RAMB_SHARED')) {
        if ($Text -notmatch $required) {
            throw "Could not apply required linker-script fix: $required"
        }
    }

    return $Text
}

$appConf = Get-Content -LiteralPath $appConfPath -Raw
$appConf = [regex]::Replace(
    $appConf,
    '#define\s+CFG_MITM_PROTECTION\s+CFG_MITM_PROTECTION_REQUIRED',
    '#define CFG_MITM_PROTECTION                   CFG_MITM_PROTECTION_NOT_REQUIRED'
)
$appConf = [regex]::Replace(
    $appConf,
    '#define\s+CFG_IDENTITY_ADDRESS\s+GAP_PUBLIC_ADDR',
    '#define CFG_IDENTITY_ADDRESS              GAP_STATIC_RANDOM_ADDR'
)
if ($appConf -notmatch 'CFG_STATIC_RANDOM_ADDRESS') {
    $appConf = [regex]::Replace(
        $appConf,
        '(#define\s+CFG_IDENTITY_ADDRESS\s+GAP_STATIC_RANDOM_ADDR\r?\n)',
        "`$1#define CFG_STATIC_RANDOM_ADDRESS         (0xC05254424C45ULL)`r`n",
        1
    )
}
$appConf = [regex]::Replace(
    $appConf,
    '#define\s+CFG_BLE_ADDRESS_TYPE\s+GAP_PUBLIC_ADDR',
    '#define CFG_BLE_ADDRESS_TYPE              CFG_IDENTITY_ADDRESS'
)
$appConf = [regex]::Replace(
    $appConf,
    'SHCI_C2_BLE_INIT_OPTIONS_GATT_CACHING_NOTUSED(?=\s*\|)',
    'SHCI_C2_BLE_INIT_OPTIONS_GATT_CACHING_USED'
)
Set-Content -LiteralPath $appConfPath -Value $appConf -NoNewline

if (Test-Path -LiteralPath $mainPath) {
    $main = Get-Content -LiteralPath $mainPath -Raw
    $main = [regex]::Replace(
        $main,
        '\r?\n\s*while\s*\(\s*LL_HSEM_1StepLock\s*\(\s*HSEM\s*,\s*CFG_HW_CLK48_CONFIG_SEMID\s*\)\s*\)\s*;',
        '',
        1
    )
    Set-Content -LiteralPath $mainPath -Value $main -NoNewline
}

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
    '(?ms)(/\* Race(?:Temp|Chrono) \*/\s*static void Custom_Can_main_Update_Char\(void\);\s*static void Custom_Can_main_Send_Notification\(void\);\s*)(?:/\* Race(?:Temp|Chrono) \*/\s*)?static void Custom_Can_main_Update_Char\(void\);\s*static void Custom_Can_main_Send_Notification\(void\);\s*',
    '$1'
)
$customApp = Remove-DuplicateFunction -Text $customApp -FunctionName 'Custom_Can_main_Update_Char'
$customApp = Remove-DuplicateFunction -Text $customApp -FunctionName 'Custom_Can_main_Send_Notification'
$customApp = [regex]::Replace(
    $customApp,
    '(?ms)/\* USER CODE BEGIN Can_main_UC_\d+\*/.*?/\* USER CODE END Can_main_UC_\d+\*/',
    "/* USER CODE BEGIN Can_main_UC_1*/`r`n`r`n  RaceChrono_CanMainUpdate();`r`n  /* USER CODE END Can_main_UC_1*/",
    1
)
$customApp = [regex]::Replace(
    $customApp,
    'Can_main_UC_\d+_Last',
    'Can_main_UC_Last'
)
$customApp = [regex]::Replace(
    $customApp,
    '(?ms)/\* USER CODE BEGIN Can_main_NS_\d+\*/.*?/\* USER CODE END Can_main_NS_\d+\*/',
    "/* USER CODE BEGIN Can_main_NS_1*/`r`n  RaceChrono_CanMainSendNotification();`r`n`r`n  /* USER CODE END Can_main_NS_1*/",
    1
)
$customApp = [regex]::Replace(
    $customApp,
    'Can_main_NS_\d+_Last',
    'Can_main_NS_Last'
)
$customApp = [regex]::Replace(
    $customApp,
    'Custom_STM_App_Update_Char\(CUSTOM_STM_CAN_MAIN, \(uint8_t \*\)UpdateCharData\);(?=\r?\n\s*\}\r?\n\s*/\* USER CODE BEGIN Can_main_NS_Last\*/)',
    'Custom_STM_App_Update_Char(CUSTOM_STM_CAN_MAIN, (uint8_t *)NotifyCharData);',
    1
)
$customApp = [regex]::Replace(
    $customApp,
    '(?:__USED\s+)*void Custom_Can_main_Send_Notification\(void\) /\* Property Notification \*/',
    '__USED void Custom_Can_main_Send_Notification(void) /* Property Notification */'
)
Set-Content -LiteralPath $customAppPath -Value $customApp -NoNewline

if (Test-Path -LiteralPath $linkerScriptPath) {
    $linkerScript = Get-Content -LiteralPath $linkerScriptPath -Raw
    $linkerScript = Repair-LinkerScript -Text $linkerScript
    Set-Content -LiteralPath $linkerScriptPath -Value $linkerScript -NoNewline
}

Write-Host 'Fixed CubeMX BLE stubs, BLE settings, and RaceTemp linker script when present.'

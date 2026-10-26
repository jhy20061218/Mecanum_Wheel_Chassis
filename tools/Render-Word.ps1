# JHY: isolated invisible Word instance, for local document layout verification.
param([Parameter(Mandatory=$true)][string]$DocumentPath,
      [Parameter(Mandatory=$true)][string]$PdfPath)
$ErrorActionPreference = 'Stop'
$sourceFile = (Resolve-Path -LiteralPath $DocumentPath).Path
$targetFile = [System.IO.Path]::GetFullPath($PdfPath)
$wordInstance = $null
$openedDocument = $null
try {
    $wordInstance = New-Object -ComObject Word.Application
    $wordInstance.Visible = $false
    $wordInstance.DisplayAlerts = 0
    $openedDocument = $wordInstance.Documents.Open($sourceFile, $false, $true)
    $openedDocument.ExportAsFixedFormat($targetFile, 17)
    Write-Output $targetFile
} finally {
    if ($null -ne $openedDocument) { $openedDocument.Close(0) }
    if ($null -ne $wordInstance) { $wordInstance.Quit(0) }
    if ($null -ne $openedDocument) { [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($openedDocument) }
    if ($null -ne $wordInstance) { [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($wordInstance) }
}

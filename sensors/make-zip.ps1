function ZipFiles( $zipfilename, $sourcedir )
{
   Add-Type -Assembly System.IO.Compression.FileSystem
   $compressionLevel = [System.IO.Compression.CompressionLevel]::Optimal
   [System.IO.Compression.ZipFile]::CreateFromDirectory($sourcedir,
        $zipfilename, $compressionLevel, $false)
}

rm -r -force temp
rm ROBBO-SENSORS.zip
mkdir temp
Get-ChildItem *\*.zip | Copy-Item -destination temp
ZipFiles ROBBO-SENSORS.zip temp
rm -r -force temp
pause
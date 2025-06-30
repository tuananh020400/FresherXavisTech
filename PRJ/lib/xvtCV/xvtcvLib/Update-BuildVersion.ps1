#>
Param(
    [ValidateNotNullOrEmpty()]
    [string]$FilePath = $null
)

#$bakFile=

#############################################################################################################
$date = Get-Date -format "yyyyMMdd-HHmmss"

$Str_Patern_Major       = "(#define\sXVT_.*VERSION_MAJOR(?!_))(.*$)"
$Str_Patern_Minor       = "(#define\sXVT_.*VERSION_MINOR(?!_))(.*$)"
$Str_Patern_Patch       = "(#define\sXVT_.*VERSION_PATCH(?!_))(.*$)"
$Str_Patern_Number      = "(#define\sXVT_.*VERSION_NUMBER(?!_))(.*$)"
$Str_Patern_Hash        = "(#define\sXVT_.*VERSION_COMIT(?!_))(.*$)"
$Str_Patern_Dirty       = "(#define\sXVT_.*VERSION_DIRTY(?!_))(.*$)"
$Str_Patern_GitBranch   = "(#define\sXVT_.*BRANCH(?!_))(.*$)"

#Get git describe
$gitBranch = git branch --show-current
$gitCommit = git log --pretty=%h -1
$gitStatus = git status --short
$gitVersionList = git tag | Where-Object { $_ -match '^v\d+\.\d+\.\d+$' }

$version     = "0.0.0"
$major_value = 0
$minor_value = 0
$patch_value = 0
$count_value = 0
$dirty_value = ""

#Get the version information
if ($gitVersionList -ne $null -and $gitVersionList.Count -gt 0)
{
    $maxVersion = "0", "0", "0"
    foreach($ver in $gitVersionList)
    {
        # Extract the version information
        $versionArray =  $ver.Substring(1).Split(".");
        $isBiger = 0

        for($i=0;$i -lt $maxVersion.count;$i++)
        {
            if([int]$maxVersion[$i] -lt [int]$versionArray[$i])
            {
                $isBiger = 1
                break;
            }
            elseif ([int]$maxVersion[$i] -gt [int]$versionArray[$i])
            {
                $isBiger = 0
                break;
            }
        }

        if($isBiger)
        {
            $maxVersion = $versionArray
            # Get the lastest version
            $version = $ver
        }
    }

    # Count how many commit from current Head to the version tag.
    $count_value = git rev-list --count "$version..HEAD"
    
    $major_value=$maxVersion[0]
    $minor_value=$maxVersion[1]
    $patch_value=$maxVersion[2]
}

if($gitStatus.Length -gt 0)
{
    $dirty_value = "dirty"
}

if ($FilePath -ne $null -and $FilePath.Length -gt 0)
{
    # Update the version to file:
    $content = Get-Content $FilePath

    # Update the major value
    $content = $content -Replace $Str_Patern_Major, ('$1 {0}' -f $major_value)

    # Update the minor value
    $content = $content -Replace $Str_Patern_Minor, ('$1 {0}' -f $minor_value)

    # Update the patch value
    $content = $content -Replace $Str_Patern_Patch, ('$1 {0}' -f $patch_value)

    # Update the pre-release number of commit value
    $content = $content -Replace $Str_Patern_Number, ('$1 {0}' -f $count_value)

    # Update the pre-release hash value
    $content = $content -Replace $Str_Patern_Hash, ('$1 {0}' -f $gitCommit)

    # Update the pre-release repo status value
    $content = $content -Replace $Str_Patern_Dirty, ('$1 {0}' -f $dirty_value)

    # Update the current git branch name
    $content = $content -Replace $Str_Patern_GitBranch, ('$1 {0}' -f $gitBranch)

    # Save updated version infor
    $content | Out-File $FilePath
}

#Debug Information
Write-Host ("- branch      : {0}" -f $gitBranch  )
Write-Host ("- commit      : {0}" -f $gitCommit  )
Write-Host ("- version     : {0}" -f $version    )
Write-Host ("- major_value : {0}" -f $major_value)
Write-Host ("- minor_value : {0}" -f $minor_value)
Write-Host ("- patch_value : {0}" -f $patch_value)
Write-Host ("- count_value : {0}" -f $count_value)
Write-Host ("- dirty_value : {0}" -f $dirty_value)

#Write-Host ($content)

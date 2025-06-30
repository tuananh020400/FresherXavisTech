set "SolutionDir=%~1"
set "ProjectDir=%~2"
set "IntermediateOutputPath=%~3"
set "ProjectName=%~4"

set "installDir=%SolutionDir%XavisTech\"
Set "includeDir=%installDir%include\%ProjectName%\"
Set "binDir=%installDir%%IntermediateOutputPath%"

Set "inputBinDir=%SolutionDir%%IntermediateOutputPath%"

::Delete the install folder
rmdir /s /q "%includeDir%"
DEL "%binDir%%ProjectName%*.lib"
DEL "%binDir%%ProjectName%*.dll"

::Make the install folder
if not exist "%includeDir%" mkdir "%includeDir%"
if not exist "%binDir%"     mkdir "%binDir%"

::Copy header files
xcopy /y /I "%ProjectDir%include\%ProjectName%\*.h"           "%includeDir%"

::Copy .lib and .dll files
xcopy /y /I "%inputBinDir%%ProjectName%*.lib"                 "%binDir%"
xcopy /y /I "%inputBinDir%%ProjectName%*.dll"                 "%binDir%"

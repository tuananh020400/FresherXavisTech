# Commenting And Document Generating {#Commenting}
Project used Doxygen to generate document from source code.  
Doxygen will generate the *static html* or *LaTeX* output.  
To generate the PDF output, should use some *LaTex* compiler to convert from *.tex* file to PDF file.
We recomend to use *MiKTeX* as the *LaTex* compiler.

## Generate Document Comment Style
1. **Configuration the Visual Studio**  
    Tools -> Options -> Text Editor -> C/C++ -> Code Style -> General -> Doxygen /**  
    <img src="VS_DocCommentStyle.png" alt="VS_DocCommentStyle Image"/>
    \image latex VS_DocCommentStyle.png "Comment Style"

2. **Comment the code follow the Doxygen rule**  
    Prefer using [C-Style comment](https://www.doxygen.nl/manual/docblocks.html).  
    Example:
    ```cplusplus
    /**
     * @brief Xavis Tech name space
    */
    namespace xvt{
    /**
     * @brief A Reference function 
    */
    void RefFunction()
    {
        std::cout << "A ref function\n";
    }

    /**
     * @brief A function to print hello world string
     * @param [in] i some number 
     * @return an int number
     * when this function relate to other class, function or other document...
     * use `@see reference` then it will create the reference link to the object automaticaly
     * @see RefFunction
    */
    int HelloWord(int i)
    {
        std::cout << "Hello world" << "i\n";
        RefFunction();
        return i;
    }
    }
    ```

##  Generate Document
1. **Doxygen**
 - Install Doxygen
 - Runing the command line in cmd: `doxygen Doxyfile`
 - Or runing by **DoxyWinzard** (Doxygen GUI frontend) App  
      + File -> Open -> Select the configuration file(**Doxyfile** ) in **xvtCV** folder  
        \image latex Doxy_OpenConfigure.png "Open Configuration File"
        <img src="Doxy_OpenConfigure.png" alt="Doxy_OpenConfigure Image"/>
      + Enter the working directory as path to the xvtCV folder
      then slect the **run** tab and click to **run doxygen** button  
        \image latex Doxy_WorkingDirectory.png "Enter working directory"
        <img src="Doxy_WorkingDirectory.png" alt="Doxy_WorkingDirectory Image"/>

2. **Install MiKTeX**  
    [MikTex](https://miktex.org) is used to generate the PDF file from Doxygen output
  - Download the latest version of MiKTex from: https://miktex.org/download.
  - Install **MiKTeX**
  - Check the **MiKTeX** update  
        Click **Check for updates** and wait for it's process  
        Click **Update now** if there are any packages need to update  
        \image latex MiKTeX_Update.png "Update MiKTeX"
        <img src="MiKTeX_Update.png" alt="MiKTeX_Update Image"/>

  - Run the command : `.\make.bat` or `.\pdflatex.exe output_name.pdf` in the **latex** folder from ouput folder of Doxygen  
    \image latex Doxy_MakePdf.png "run make.bat"
    <img src="Doxy_MakePdf.png" alt="Doxy_MakePdf Image"/>

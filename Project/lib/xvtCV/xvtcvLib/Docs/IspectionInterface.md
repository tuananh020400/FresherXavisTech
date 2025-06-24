# Inspection Interface {#IspectionInterface}
**xvtCV** provides xvt::IInspectionResult and xvt::IInspection classes that define 
interface functions for handling image inspection results and the inspection process such as: inspect, load, save, DrawResult...

These interfaces (IInspectionResult and IInspection) are designed to be flexible, extensible, 
and to promote a clean and maintainable architecture. Below is an overview of these interfaces, 
how to use them, and the benefits of using interfaces in your code.

## Abstract class introduce.
Let define a virtual class **ITest** this class has 4 virual functions.
```cplusplus
//Itest.h
#include <iostream>
class XVT_EXPORTS ITest
{
public:
    virtual ~ITest() {}

    virtual void Func1()=0;
    virtual void Func2();
};

//Itest.cpp
void ITest::Func2()
{
    std::cout << "Version 1 Func4 ITest\n";
}
```

A class **Test1** that inherited from **ITest**
and implement all pure virtual functions defined in **ITest**
```cplusplus
//Test1.h
#pragma once
#include "ITest.h"

class XVT_EXPORTS Test1 :
    public ITest
{
public:
    Test1() = default;
    // Inherited via ITest
    virtual void Func1() override;

    ~Test1() {}
    int a = 0;
    int b = 0;
    int c = 0;
};

//Test1.cpp
#include "Test1.h"
#include <iostream>

void Test1::Func1()
{
    std::cout << a << " Func1 in test1\n";
}
```

Build the **Test.dll** from **Itest** and **Test1**.  
Using that Test.dll in other project call **TestInterface**  
```cplusplus
//TestInterface.cpp
#include <iostream>
#include "../Project1/ITest.h"
#include "../Project1/Test1.h"

class SomeClass
{
public:
    // Use as a member
    Test1 instance = Test1();
};

int main()
{
    std::cout << "Use as a member!\n";
    SomeClass a;

    a.instance.Func1();
    a.instance.Func2();

    return 0;
}
```

Building and runing the programe **TestInterface.exe**.  
Output of that programe:  
```cplusplus
Use as a member!
0 Func1 in test1
Version 1 Func2 ITest
```

Then now, we want to implement the **Func2** functions in **Test1** class as follow.  
```cplusplus
//Test1.h
#pragma once
#include "ITest.h"

class XVT_EXPORTS Test1 :
    public ITest
{
public:
    Test1() = default;
    // Inherited via ITest
    virtual void Func1() override;
    virtual void Func2() override;

    ~Test1() {}
    int a = 0;
    int b = 0;
    int c = 0;
};

//Test1.cpp
#include "Test1.h"
#include <iostream>

void Test1::Func1()
{
    std::cout << a << "Version 2 Func1 in test1\n";
}

void Test1::Func2()
{
    std::cout << c << "Version 2 Func2 test1\n";
}
```

Build the **Test.dll** and copy to the folder contains **TestInterface.exe** without rebuild **TestInterface.exe**.
Output of that programe:  
```cplusplus
PS G:\tmp\TestInterface\x64\Debug> .\TestInterface.exe
Use as a member!
0Version 2 Func1 in test1
Version 1 Func2 ITest
```

**Func1** is running with new implementation **Version 2** but **Func2** still linked to 
implementation of **ITest** class. 
Because in the first dll, there are no information of **Func2** at **Test1** class
so compiler linked that function to **ITest** class.

Now, we want change more to **Test1** class by add one more memmer call **otherMember**. 
```cplusplus
//Test1.h
#pragma once
#include "ITest.h"

class XVT_EXPORTS Test1 :
    public ITest
{
public:
    Test1() = default;
    // Inherited via ITest
    virtual void Func1() override;

    ~Test1() {}
    int a = 0;
    int b = 0;
    int c = 0;
    float otherMember = 0;
};
```
Build the **Test.dll** and copy to the folder contains **TestInterface.exe** without rebuild **TestInterface.exe**.
An error happened.  
<img src="interface_error.png" alt="Ver2W2B Image"/>

It becauses we change the class **Test1** structure, it now has **otherMember**
the memmory allcation for it is changed then in runtime it accessed wrong position so it is corrupted.

How to overcome that issue.  
Using as pointer.  

```cplusplus
//TestInterface.cpp
#include <iostream>
#include "../Project1/ITest.h"
#include "../Project1/Test1.h"

class SomeClass
{
public:
    // Use as a pointer.
    std::unique_ptr<ITest> ptr = std::make_unique<Test1>();
};

int main()
{
    std::cout << "Use as a pointer!\n";
    SomeClass a;

    a.ptr->Func1();
    a.ptr->Func2();

    return 0;
}
```

Output of that programe:  
```cplusplus
PS G:\tmp\TestInterface\x64\Debug> .\TestInterface.exe
Use as a pointer!
0Version 2 Func1 in test1
0Version 2 Func2 test1
```
**Note:**
- Always using the interface as a smart pointer so we can have "Couple Loosing".
- If the child class did not override the parrent interface fucntion in the previous version,
    then when you release the new version that introduce the new implement of that function in child class
    you have to notice the user that it need rebuild.

## Overview of xvt::IInspectionResult Interface

The xvt::IInspectionResult interface encapsulates the results of an image inspection process. 
It provides methods for:

 - Setting and getting the inspection result (EResult).
 - Handling messages related to the inspection process.
 - Drawing results and messages on images.
 - Managing regions of interest (ROI) in the inspected image.
 - Cloning the inspection result object.
 - Combining results from multiple inspections.

**Example: Using IInspectionResult**  
Here's how you might use the IInspectionResult interface in your code:  
```cplusplus
class MyInspectionResult : public xvt::IInspectionResult
{
    // Implement all pure virtual functions defined in IInspectionResult
};

std::unique_ptr<xvt::IInspectionResult> result = std::make_unique<MyInspectionResult>();

// Set and get the result
result->SetResult(xvt::EResult::OK);
auto res = result->GetResult();

// Set a message
result->SetMsg("Inspection completed successfully");

// Draw the result on an image
cv::Mat image = cv::imread("test.jpg");
result->DrawResult(image);

// Clone the result object
auto clonedResult = result->Clone();
```

## Overview of xvt::IInspection Interface
The IInspection interface represents the image inspection process. 
It provides methods for:

 - Inspecting single or multiple images.
 - Loading and saving inspection data from/to a file.
 - Cloning the inspection object.

**Example: Using IInspection**  
Here's how you might use the IInspection interface in your code:
```cplusplus
class MyInspection : public xvt::IInspection
{
    // Implement all pure virtual functions defined in IInspection
};

std::unique_ptr<xvt::IInspection> inspection = std::make_unique<MyInspection>();

// Inspect a single image
cv::Mat image = cv::imread("test.jpg");
auto result = inspection->Inspect(image);

// Inspect multiple images
std::vector<cv::Mat> images = {image1, image2, image3};
auto result = inspection->Inspect(images);

// Save and load inspection data
inspection->Save("inspection_data.xml");
inspection->Load("inspection_data.xml");

// Clone the inspection object
auto clonedInspection = inspection->Clone();
```

## Why Use Interfaces as Virtual Classes?
Using interfaces like IInspection and IInspectionResult as virtual classes has several advantages:
1. Flexibility and Extensibility:
    By defining the interfaces as virtual classes, you can easily extend the functionality by creating new classes that implement these interfaces. 
    This allows for different inspection algorithms or result-handling strategies to be developed independently without changing the core interface.
2. Separation of Concerns: Interfaces help in separating the definition of the operations from their implementation. 
   This means that the code responsible for performing inspections can be written independently from the code that uses these inspections.
3. Reusability: Interfaces promote reusability by allowing different parts of a program to interact with each other through well-defined contracts, 
    without needing to know the specifics of the implementation.
4. Testability: Interfaces make it easier to write unit tests by allowing mock implementations to be used in place of real implementations during testing. 
    This makes it possible to test components in isolation.
5. Loose Coupling: By relying on interfaces rather than concrete classes, your code becomes less dependent on specific implementations, 
   making it easier to modify or replace components in the future.

## Conclusion
The xvt::IInspection and xvt::IInspectionResult interfaces provide a powerful framework for image inspection tasks. 
They offer a clear, extendable, and maintainable way to manage inspection processes and results, promoting good software design practices such as separation of concerns, flexibility, and testability.

Implementing and using these interfaces in your projects will help create robust, adaptable, and maintainable software solutions for image inspection tasks.
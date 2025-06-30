# Saving Data to File by CSV format {#SupportCSV}
**xvtCV** provides xvt::CSV class that define 
interface functions for handling saving data to file in CSV format.

## How CSV Saved in the File.
xvt::CSV class supports many functions that handling converting data to string and saving to the file.
The two main functions: xvt::CSV::Save() and xvt::CSV::GetCSVData()  
In Save function, it internally call xvt::CSV::GetCSVData to get the output data. Then from 
that data it extract the header and data information.
If `isNewFile == true` it will create the new file and add the tile tile and the header row.  
if not it will only append the data to the end of file.

## Step-by-Step Guide
Assume that we have a inspection result class **xvt::SomeResult** and we want to save
its' data to a **.csv** file.  

@dontinclude test/xvtCSVTest.cpp
@skipline class SomeResult
@skipline {
@until public:
@skipline Class member variables
@until };

First, we need to inherit from xvt::CSV class then override function  

@snippet test/xvtCSVTest.cpp Define SomeResult Class

we save all SomeResult's member data to csv file.
Then when calling to SomeResult::Save it will save those data to csv file.  

|Int Data|Float Data|Double Data|String Data|
|--------|----------|-----------|-----------|
|      10|   1.32324|       1.23|   A String|

if we want to put addition title to the result we can override function:  

@snippet test/xvtCSVTest.cpp Define SomeResult Class GetTitle

## Complete Example Code

@snippet test/xvtCSVTest.cpp Example CSV support
Outpute of file *test_save_csv_class.csv*:  
```cplusplus
A Additional Title
Int Data,Float Data,Double Data,String Data
10,1.323240,1.230000,A String
```

@see  xvt::CSV, xvtCSVTest.cpp
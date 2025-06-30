# Signal Processing and Peak/Valley Detection in Images {#PeakFinding}

**xvtCV** provides xvt::FindPeaks class that support finding the peak/valley.

## Peak and valley Algorithm

The peak/valley finding algorithm concept is based on prominence measurement.

The [prominence](https://www.mathworks.com/help/signal/ug/prominence.html) of a peak measures how much the peak stands out due to its intrinsic height
and its location relative to other peaks. A low isolated peak can be more prominent than one that is higher but is an otherwise unremarkable member of a tall range.

**To measure the prominence of a peak:**

1.  Place a marker on the peak.
2.  Extend a horizontal line from the peak to the left and right until the line does one of the following:
    - Crosses the signal because there is a higher peak
    - Reaches the left or right end of the signal
3.  Find the minimum of the signal in each of the two intervals defined in Step 2. This point is either a valley or one of the signal endpoints.
4.  The higher of the two interval minima specifies the reference level. The height of the peak above this level is its prominence.

Peak finder makes no assumption about the behavior of the signal beyond its endpoints, whatever their height. 
As a result, Steps 2 and 4 disregard signal behavior beyond endpoints, which often affects the value of the reference level. 
Consider for example the peaks of this signal: Signal with nine peaks, numbered 1 through 9 from left to right. 
The valleys between each pair of peaks are labeled from left to right with the letters a through i. 
In decreasing order of height(value), the peaks are 2, 6, 1, 8, 4, 7, 3, 9, and 5. 
In decreasing order of height(value), the valleys are a, g, c, i, b, f, e, d, and h.  
![Signal](peaksnolines.png)  
\image latex peaksnolines.png "signal"

| Peak Number | Left Interval Lies Between Peak and | Right Interval Lies Between Peak and | Lowest Point on the Left Interval | Lowest Point on the Right Interval | Reference Level (Highest Minimum) |
|-------------|-------------------------------------|--------------------------------------|-----------------------------------|------------------------------------|-----------------------------------|
| 1           | Left end                            | Crossing due to peak 2               | Left endpoint                     | a                                  | a                                 |
| 2           | Left end                            | Right end                            | Left endpoint                     | h                                  | Left endpoint                     |
| 3           | Crossing due to peak 2              | Crossing due to peak 4               | b                                 | c                                  | c                                 |
| 4           | Crossing due to peak 2              | Crossing due to peak 6               | b                                 | d                                  | b                                 |
| 5           | Crossing due to peak 4              | Crossing due to peak 6               | d                                 | e                                  | e                                 |
| 6           | Crossing due to peak 2              | Right end                            | d                                 | h                                  | d                                 |
| 7           | Crossing due to peak 6              | Crossing due to peak 8               | f                                 | g                                  | g                                 |
| 8           | Crossing due to peak 6              | Right end                            | f                                 | h                                  | f                                 |
| 9           | Crossing due to peak 8              | Crossing due to right endpoint       | h                                 | i                                  | i                                 |

## Example:  
This example demonstrates how to process a signal extracted from signal above.  
![Result Image Distance 5](peaksnolineres5.png)  
\image latex peaksnolineres5.png "Result Image Distance 5"  
Since the origin of the image is at the top-left corner, the blue plus sign indicates the valleys, and the red plus sign indicates the peaks.  

@snippet test/FindPeakTest.cpp Peak Finding Examples

In the example code, we filtered out peaks and valleys with a prominence greater than 10, ensuring that the distance between each is at least 5.
```c
    int pro = 10;
    int dis = 5;
...
    //! Find the peak that has prominence > 10.
    auto peaks = peakFinder.GetPeakResult(pro, dis);
...
    //! Find the valley that has prominence < -10.
    auto valleys = valleyFinder.GetPeakResult(pro, dis);
```

If dis=0, it means all peaks and valleys will be considered, which could result in multiple peaks at the same location.  
![Result Image Distance 0](peaksnolineres0.png)  
\image latex peaksnolineres0.png "Result Image distance 0"  

## More information
See xvt::FindPeaks, FindPeakTest.cpp
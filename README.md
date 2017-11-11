# IEQbot: Towards high-granularity agile indoor environmental quality monitoring

Indoor environmental quality (IEQ) is a critical aspect of the built environment to ensure occupant health, comfort, well-being and productivity. Existing IEQ monitoring approaches rely on sensor networks deployed at selected locations to collect environmental measurements, and are limited in scale and adaptability due to infrastructure cost and maintenance requirement. *To enable high-granularity IEQ monitoring with agile adaption to the dynamic indoor environment, we propose an “automated mobile sensing” system that dispatches a sensor-rich navigation-capable robot to actively survey the indoor space.* 

Data collected in this fashion is sparse in the joint temporal and spatial domain, and cannot be used directly for IEQ evaluation. To deal with this special characteristics, we developed a spatio-temporal interpolation algorithm to capture the global trend and local variation in order to use the data efficiently to reconstruct the IEQ dynamics. We compared the performance of the automated mobile sensing with a dense sensor network in a laboratory where we measured the air-change effectiveness (ASHRAE standard 129) for four different conditions. Results indicate that automated mobile sensing is able to accurately estimate the parameters with a minimal sensor cost and calibration effort. 

Potential applications of this system include indoor thermal comfort, lighting, indoor air quality and acoustic monitoring, pollutant source identification, and building commissioning. We shared publicly the source codes for robot control, sensor setup, and interpolation algorithm to encourage comparison study and further development.

Feel free to use it! Please contact [Ming Jin](http://www.jinming.tech/) for any enquiries.

Contributors: Yulun Tian, Mingjian Lu, Ming Jin


## Reference

[Journal] Ming Jin, Shichao Liu, Stefano Schiavon, Costas Spanos, "Automated mobile sensing: Towards high-granularity agile indoor environmental quality monitoring", Building and Environment, Available online 6 November 2017, ISSN 0360-1323, https://doi.org/10.1016/j.buildenv.2017.11.003. 
[Conference] Ming Jin, Shichao Liu, Yulun Tian, Mingjian Lu, Stefano Schiavon, and Costas Spanos, "Indoor environmental quality monitoring by autonomous mobile sensing", [ACM BuildSys, 2017, Delft, Netherlands](http://buildsys.acm.org/2017/program/)

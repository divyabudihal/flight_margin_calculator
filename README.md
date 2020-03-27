# Flight Margin

This library calculates the energy margin for a flight. `FlightMargin::RemainingBattery()` takes in initial battery level, waypoints, winds, airspeed, and required power for the airspeed to output the remaining battery energy after the flight. The set of waypoints is discretized into small segments, and by calculating windspeed vectors for each of these segments, we calculate a groundspeed vector. With this, we can find the time required to fly the entire mission, thus allowing us to calculate the energy required to fly the mission.

This library uses the Ordinary Kriging spatial interpolation method for computing wind vectors at specific locations. The Kriging model with a spherical semivariogram model was shown to be the most effective in accurately interpolating wind data [1]. The algorithm is further described [here](http://spatial-analyst.net/ILWIS/htm/ilwisapp/kriging_algorithm.htm).

The configuration parameters for `semivariogram` in `config/flight_margin.yaml` are based on a previous study's results when examining data in England and Wales [1]. The other `discretization_frequency_hz: 0.1` setting was based on an approximation based on a reasonable global planner frequency. No major performance improvement was seen when this frequency was increased.

This library also plots distance travel vector data and windspeed vector data. Example figure are shown below:

**Test Data with Small Values:**
![](/images/test_travel_vectors.png)
![](/images/test_windspeed_vectors.png)

**Real Waypoints and Wind Data in UTM x and y coordinates**
![](/images/utm_waypoint_travel_vectors.png)
![](/images/utm_windspeed_vectors.png)


1. Luo, W., et al. “A Comparison of Spatial Interpolation Methods to Estimate Continuous Wind Speed Surfaces Using Irregularly Distributed Data from England and Wales.” International Journal of Climatology, vol. 28, no. 7, 2008, pp. 947–959., doi:10.1002/joc.1583.
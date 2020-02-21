# **WHEEL ODOMETRY PUBLISHER**

*Package contains node that calculates and publishes odometry based on data from wheel encoders such as traveled distance in given iteration or (in case of robots which wheels can rotate along z-axis) current angle with base_link frame. It works for any (>1) number of wheels and also does not need data from same number of wheels in every iteration (which means that after failure of some encoders algorithm is still capable to calculate odometry)*


**Useful info**  
* Every length / distance is specified in meters and every angle in radians.

* Algorithm is capable of ignoring noisy wheels (only in case of robots which wheels can rotate along z-axis)

**Data** 
  
* shifts of wheels from one iteration must be published on /wheel_shifts topic as WheelShifts message which is defined in this package. 


**Launching** 
 
* To launch node just build this package and run wheel_odom.launch file 

**config.yaml parameters**
    
  * filter_noisy_wheels - (True/False) works only for robots which wheels can turn along z-axis. leave it false for robots with differential drive. 

  * noisy_wheels_filter_threshold - (float) works only if above is set to True. It is measure of error, which is defined for every wheel as sum of (Dd / Dt) 
                                            Where Dd - relative displacement between this wheel and other wheel
                                                  Dt - distance traveled by wheel in this iteration.
                                        In perfect situation Dd between any two wheels is zero and Dd > 0 means slippage.
                                        If above error is bigger than noisy_wheels_filter_threshold then wheel is considered to be unreliable and is skipped 
                                        in this iteration 

  * min_number_of_reliable_wheels - (int) if number of reliable wheels is lower than that then odometry is sent with infinite covariance. Do not set it below 2
    
**wheels.yaml  parameters**

* in this file one can specify location of wheels w.r.t. base_link in the manner shown in that file. Algorithm give no        restriction on number or location of wheels. Coordinate frame used in this config is shown below. 

        ############################################
        #       ##              #           ##     # 
        #     ##fl##         X  #         ##fr##   #
        #       ##              #           ##     #
        #                       #                  #
        #                       #                  #
        #                       #                  #
        #                       #                  #
        #     # # # # # # # # # #                  #
        #       Y                                  #
        #                                          #
        #                                          #
        #       ##                          ##     #
        #     ##bl##                      ##br##   #
        #       ##                          ##     #
        ############################################



float64 GNSS_time         # The GNSS time of week of the observation data, in seconds.
float64 doppler_shifts  # An array of Doppler shifts for each satellite, in Hz.
float64 lambdas         # An array of wavelengths for each satellite, in meters.
int32 n                   # The number of observation data points.
float64[] rs              # An array of satellite positions and velocities in ECEF coordinates, in meters and meters per second.
                          # The array length should be 6 * n, with each satellite's data in the format [x,y,z,vx,vy,vz].
float64[] dts             # An array of satellite clock biases and drifts, in seconds.
                          # The array length should be 2 * n, with each satellite's data in the format [dt, dtr].
float64[] rr              # The receiver's position in ECEF coordinates, in meters. The array length should be 3.
float64[] azel            # An array of azimuth and elevation angles for each satellite, in radians.
                          # The array length should be 2 * n, with each satellite's data in the format [azimuth, elevation].
int32 vsat              # An array of flags indicating whether each satellite is valid. The array length should be n.
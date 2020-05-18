# Each MeasurementPackage must have the following definitions
# - a Measurement with meas_type="final_time" indicating the final time measurements were considered
# for this package
# - a measurement stream start, a "bookstart". This can be indicated by any explicit measurement OR by a "*_bookstart"
# measurement type. Either of these tell ETDDF to start filling in implicit measurements after this time
# - Optionally, a measurement stream stop, a "bookend". These indicate that a measurement stream stopped, such as an
# asset being no longer in view of the asset's sonar. These are indicated by meas_type="*_bookend". If the
# measuremnt stream continues through the end of "final_time" a bookend is not needed.

etddf/Measurement[] measurements
string src_asset
float32 delta_multiplier
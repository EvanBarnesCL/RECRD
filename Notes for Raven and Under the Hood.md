Notes for Raven and Under the Hood


# Sensor Notes
The sensors get polled every 50ms in stock code, or 20 times per second.
k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);      // control how frequently we poll the sensors on the I2C bus (color sensor, both encoders)
I2C_UPDATE_INTERVAL is 50
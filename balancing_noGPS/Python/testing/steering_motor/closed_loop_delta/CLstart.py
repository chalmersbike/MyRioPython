"""
This test matches SteeringReference.slx
It holds all state errors except delta at zero
After a specified time delay, it gives a step input to delta_reference
The resuls are logged
"""

from CLbike import Bike

bike = Bike(debug=False)

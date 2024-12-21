This was created to follow along with the Ros2 tutorials, this one in particular is now showing how I can build a custom interface, or message.

Defined two new message objects
1. msg/Num.msg    - This one just includes a base numerical object
2. msg/Sphere.msg - This one looks to import a predefined ros object Point as well as using a base type float.

Defined a service definition
1. AddThreeInts.srv - Values above the "---" are expected inputs, where as below that line are expected response

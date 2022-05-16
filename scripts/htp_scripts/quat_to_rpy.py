import tf

x = -0.11814462512022551
y = -0.0007927624820192815
z = -0.9929737278063924
w = 0.0066629549692623375


quaternion = (x,y,z,w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

print(f"r: {roll}\np: {pitch}\ny: {yaw}")
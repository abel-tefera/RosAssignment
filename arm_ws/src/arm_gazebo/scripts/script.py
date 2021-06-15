# import numpy as np

# def rotationX(x):
#     thetaX = np.radians(x)
#     rotX = np.array([
#         [1, 0, 0, 0],
#         [0, np.cos(thetaX), -np.sin(thetaX), 0],
#         [0, np.sin(thetaX), np.cos(thetaX), 0], 
#         [0, 0, 0, 1]
#     ], dtype=np.float16)
#     return rotX

# def rotationY(y):
#     thetaY = np.radians(y)
#     rotY = np.array([
#         [np.cos(thetaY), 0, np.sin(thetaY)],
#         [0, 1, 0],
#         [-np.sin(thetaY), 0, np.cos(thetaY)],
#         [0, 0, 0, 1]

#     ])
#     return rotY

# def rotationZ(z): 
#     thetaZ = np.radians(z)
#     rotZ = np.array([
#         [np.cos(thetaZ), -np.sin(thetaZ), 0, 0],
#         [np.sin(thetaZ), np.cos(thetaZ), 0, 0],
#         [0, 0, 1, 0],
#         [0, 0, 0, 1]
#     ])
#     return rotZ

# def translationZ(a = 0, b = 0, c = 0):
#     traZ = np.array([
#         [1, 0, 0, a],
#         [0, 1, 0, b],
#         [0, 0, 1, c],
#         [0, 0, 0, 1],
#     ])
#     return traZ

# def rotXTransZ(V, x, z):
#     return (V.dot(rotationX(x))).dot(translationZ(z))

# # Tz 0.1 Rz 60 Tz 0.05 Rx 30 Tz 2.0 Rx -90 Tz 1.0 Rx -30 Tz 1.0

# # V = np.array([0, 0, 0, 1])
# V_p = ((((((((translationZ(0.1)
# .dot(rotationZ(60)))
# .dot(translationZ(0.05)))
# .dot(rotationX(30)))
# .dot(translationZ(2.0)))
# .dot(rotationX(-90)))
# .dot(translationZ(1.0)))
# .dot(rotationX(-30)))
# .dot(translationZ(1.0)))

# # print (rotXTransZ(np.array([1, 2, 3, 1]), 1.57, 4))
# # print(V_p)

# import tinyik as ik

# arm = ik.Actuator([
#     'z', [0, 0, 0.15],
#     'x', [0, 0, 2.0],
#     'x', [0, 0, 1],
#     'x', [0, 0, 1]
# ])

# arm.ee = [1, 1, 1]

# print (arm.angles)
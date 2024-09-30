from construct import *
from construct import Int32ul, Float32l

# head_SendRadRotationAndPosition = Struct(
# 	Const(2, Int32ul),		# Version
# 	Const(3, Int32ul),	# SendRadRotationAndPosition 
# 	Const(24, Int32ul),		# DataLength
# 	"rotation" / Array(3, Float32l),
# 	"position" / Array(3, Float32l),
# 	Padding(40),
# )

# print(head_SendRadRotationAndPosition)


# # SendRadRotationAndPosition
# anglesposition = Struct(
# 	Const(2, Int32ul),  # Version
# 	Const(3, Int32ul),  # SendRadRotationAndPosition
# 	Const(24, Int32ul), # DataLength
# 	"data" / Padded(64, Array(6, Float32l)),
# )

# print(anglesposition)

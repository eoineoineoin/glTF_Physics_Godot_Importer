extends Node3D
class_name KHR_CollisionShape

@export var shape : Shape3D

# This is a container very similar to CollisionShape3D, which 
# does not need to be the immediate child of a CollisionObject3D
# (i.e., will work for a grandparent CollisionObject3D)
#<todo.eoin Need to listen for transform-changed signals
func _ready():
	if shape == null:
		return

	var bodyFromShape : Transform3D = transform

	var parent : Node3D = get_parent()
	while parent != null:
		if parent as CollisionObject3D:
			break
		bodyFromShape = parent.transform * bodyFromShape
		parent = parent.get_parent()

	if parent != null:
		var body : CollisionObject3D = parent as CollisionObject3D
		var ownerId = body.create_shape_owner(self)
		body.shape_owner_add_shape(ownerId, shape)
		body.shape_owner_set_transform(ownerId, bodyFromShape)

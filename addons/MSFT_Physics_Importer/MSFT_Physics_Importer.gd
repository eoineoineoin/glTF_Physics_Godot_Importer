@tool
extends GLTFDocumentExtension
class_name MSFT_Physics

const extensionName : String = "MSFT_rigid_bodies"
const collisionPrimitivesExtension : String = "MSFT_collision_primitives"
const c_ext : String = "extensions"

class PerDocumentPhysicsData:
	# Additional information per-GLTFDocument that we need to construct physics objects
	var nodeToGltfNodeMap : Dictionary #todo.eoin Don't need this. Can use GLTFState.get_scene_node()

class PerNodePhysicsData:
	# Additional information per-GLTFNode that we need to construct physics objects
	var extensionData : Dictionary

func _fixupNodeOwners(curNode : Node, owner : Node):
	# We have added some nodes to the middle of the scene tree.
	# Recursively walk the tree and set their owner so that the nodes aren't dropped
	# from the PackedScene. We need to this after the tree has been reconstructed as
	# you can't set_owner() until after your node is a child of the owner.
	for c in curNode.get_children():
		_fixupNodeOwners(c, owner)

	if curNode != owner:
		curNode.set_owner(owner)

func postSceneConvert(state : GLTFState, root : Node) -> Node:
	# Recursively traverse root and create colliders, bodies, and joints
	var docData : PerDocumentPhysicsData = state.get_additional_data(extensionName)
	var newRoot : Node = _recurseCreateCollidersAndBodies(state, docData, root, null)
	newRoot = _recurseCreateJoints(state, docData, newRoot, newRoot)
	# newRoot *should* equal root, as it's a container for the whole scene
	_fixupNodeOwners(newRoot, root)
	return newRoot

func _recurseCreateJoints(state : GLTFState, docData : PerDocumentPhysicsData, curNode : Node3D, rootNode : Node3D) -> Node3D:
	for c in curNode.get_children():
		_recurseCreateJoints(state, docData, c, rootNode)

	var outputNode = curNode
	if docData != null and docData.nodeToGltfNodeMap.has(curNode):
		var gltfNode : GLTFNode = docData.nodeToGltfNodeMap[curNode]
		var perNodeData : PerNodePhysicsData = gltfNode.get_additional_data(extensionName)
		if perNodeData != null:
			if perNodeData.extensionData.has("joint"):
				var jointData = perNodeData.extensionData["joint"]
				var jointNode : Generic6DOFJoint3D = _constructJointLimits(jointData)
				# We can just add the joint node as a child of curNode, which will provide the
				# (single) pivot in world space
				curNode.add_child(jointNode)

				# Now, hook up the connected bodies.  curNode will become constraint bodyA,
				# while connectedNode will be bodyB.
				var connectedGlTFNodeIdx : int = jointData["connectedNode"]
				var connectedNode : Node3D = state.get_scene_node(connectedGlTFNodeIdx)
				var bodyANode = _getParentBody(curNode)
				var bodyBNode = _getParentBody(connectedNode)
				jointNode.node_a = jointNode.get_path_to(bodyANode)
				jointNode.node_b = jointNode.get_path_to(bodyBNode)

	return outputNode

func _getParentBody(node : Node3D) -> PhysicsBody3D:
	# For a given node, return the first parent PhysicsBody3D
	while node != null:
		if node is PhysicsBody3D:
			return node
		node = node.get_parent()
	return null

func _constructJointLimits(jointData : Dictionary) -> Generic6DOFJoint3D:
	var joint : Generic6DOFJoint3D = Generic6DOFJoint3D.new()
	joint.name = "Joint"
	joint.exclude_nodes_from_collision = !jointData.has("enableCollision") or jointData["enableCollision"] == false

	# First, enable all the DOFs
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, false)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, false)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, false)
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, false)

	for limit in jointData["constraints"]:
		var isLocked = !(limit.has("min") or limit.has("max"))
		var minLimit = limit["min"] if limit.has("min") else -1e38 #todo.eoin is there an FLT_MIN/FLT_MAX?
		var maxLimit = limit["max"] if limit.has("max") else 1e38
		# todo.eoin There is probably a cleaner way to write this:
		if limit.has("linearAxes"):
			for axisIdx in limit["linearAxes"]:
				if axisIdx == 0:
					joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
					if not isLocked:
						joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, minLimit)
						joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, maxLimit)
				if axisIdx == 1:
					joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
					if not isLocked:
						joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, minLimit)
						joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, maxLimit)
				if axisIdx == 2:
					joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
					if not isLocked:
						joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, minLimit)
						joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, maxLimit)
		if limit.has("angularAxes"):
			for axisIdx in limit["angularAxes"]:
				if axisIdx == 0:
					joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
					if not isLocked:
						joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, minLimit)
						joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, maxLimit)
				if axisIdx == 1:
					joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
					if not isLocked:
						joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, minLimit)
						joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, maxLimit)
				if axisIdx == 2:
					joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
					if not isLocked:
						joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, minLimit)
						joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, maxLimit)
	return joint

func _recurseCreateCollidersAndBodies(state : GLTFState, docData : PerDocumentPhysicsData, curNode : Node3D, parentBody : CollisionObject3D) -> Node3D:
	# Need to unhook all the children, as we may need to be able to change their parent
	var originalChildren : Array[Node3D]
	originalChildren.append_array(curNode.get_children())
	for c in originalChildren:
		curNode.remove_child(c)

	# Need to construct one of three hierarchies:
	# If we see a rigidBody -> make new Rigid body, add curNode child
	# If we see a collider and we have a rigid body -> add shape to body, descend & return curNode
	# If we see a collider and no rigid body set -> create new static body, add shape, add curNodeChild

	var outputNode : Node3D = curNode # We may need to add an additional parent to curNode
	var newChildren : Array[Node3D]

	if docData != null and docData.nodeToGltfNodeMap.has(curNode):
		var gltfNode : GLTFNode = docData.nodeToGltfNodeMap[curNode]
		# todo.eoin This is incorrect for joint nodes, but a joint would either already be a child
		# of a physics collider or the joint would be constrained to the "world".
		var perNodeData : PerNodePhysicsData = gltfNode.get_additional_data(extensionName)
		if perNodeData != null and perNodeData.extensionData.has("rigidBody"):
			parentBody = createRigidBody(perNodeData.extensionData["rigidBody"])
			parentBody.name = str(curNode.name, "_rigidBody")
			parentBody.transform = curNode.transform
			parentBody.add_child(curNode)
			curNode.transform = Transform3D.IDENTITY
			outputNode = parentBody
		if perNodeData != null and perNodeData.extensionData.has("collider"):
			if parentBody == null:
				parentBody = StaticBody3D.new()
				parentBody.name = str(curNode.name, "_staticBody")
				parentBody.transform = curNode.transform
				parentBody.add_child(curNode)
				curNode.transform = Transform3D.IDENTITY
				outputNode = parentBody

			# Finally, add the collider to the parent body
			# Seems that if we use parentBody.create_shape_owner() now, the stored shapes will
			# be lost when converted to a PackedScene. Instead, we have to add a CollisionShape3D
			# which contains our shape resource.
			var collider = createColliderObject(state, perNodeData.extensionData["collider"])
			curNode.add_child(collider)


	for c in originalChildren:
		newChildren.append(_recurseCreateCollidersAndBodies(state, docData, c, parentBody))

	for c in newChildren:
		outputNode.add_child(c)

	return outputNode

func _get_supported_extensions():
	return [extensionName]

func _parse_node_extensions(state : GLTFState, gltfNode : GLTFNode, extensions : Dictionary):
	if not extensions.has(extensionName):
		return
	# Some of the shapes require us to read the mesh data; however, at this point in the pipeline,
	# state.get_meshes() returns [], so instead, we'll save the extension info on the gltfNode
	# and revisit it later.
	var perNodeData : PerNodePhysicsData = PerNodePhysicsData.new()
	perNodeData.extensionData = extensions[extensionName]
	gltfNode.set_additional_data(extensionName, perNodeData)

func _import_preflight(state: GLTFState, extensions: PackedStringArray) -> int:
	state.set_additional_data(extensionName, PerDocumentPhysicsData.new())
	return OK

func _import_node(state : GLTFState, gltfNode : GLTFNode, jsonData : Dictionary, node : Node) -> int:
	var documentPhysics : PerDocumentPhysicsData = state.get_additional_data(extensionName)
	documentPhysics.nodeToGltfNodeMap[node] = gltfNode
	return OK

func createRigidBody(jsonData : Dictionary) -> RigidBody3D:
	var rigidBody : RigidBody3D = RigidBody3D.new()
	if "mass" in jsonData:
		rigidBody.mass = jsonData["mass"]
	else:
		rigidBody.mass = 1

	if "linearVelocity" in jsonData:
		rigidBody.linear_velocity = _arrayToVector3(jsonData["linearVelocity"])
	if "angularVelocity" in jsonData:
		rigidBody.angular_velocity = _arrayToVector3(jsonData["angularVelocity"])
	return rigidBody

func createColliderObject(state : GLTFState, colliderIndex : int) -> MSFT_CollisionShape:
	var collisionShape = MSFT_CollisionShape.new()
	var jsonData = state.json.extensions[collisionPrimitivesExtension].colliders[colliderIndex]
	var shape = createColliderShape(state, jsonData)
	collisionShape.name = "MSFT_CollisionShape"
	collisionShape.shape = shape
	shape.reference()
	return collisionShape

func createColliderShape(state : GLTFState, jsonData : Dictionary) -> Shape3D:
	if jsonData.has("sphere"):
		return makeSphereShape(jsonData["sphere"])
	if jsonData.has("box"):
		return makeBoxShape(jsonData["box"])
	if jsonData.has("capsule"):
		return makeCapsuleShape(jsonData["capsule"])
	if jsonData.has("cylinder"):
		return makeCylinderShape(jsonData["cylinder"])
	if jsonData.has("convex"):
		return makeConvexShape(state, jsonData["convex"])
	if jsonData.has("trimesh"):
		return makeTriMeshShape(state, jsonData["trimesh"])
	print_debug("Unhandled collider type", jsonData)
	return null

func makeSphereShape(sphereData : Dictionary) -> Shape3D:
	var sphereShape : SphereShape3D = SphereShape3D.new()
	sphereShape.radius = sphereData["radius"]
	return sphereShape

func makeBoxShape(boxData : Dictionary) -> Shape3D:
	var boxShape : BoxShape3D = BoxShape3D.new()
	boxShape.size = _arrayToVector3(boxData["size"])
	return boxShape

func makeCapsuleShape(capsuleData : Dictionary) -> Shape3D:
	var capsuleShape : CapsuleShape3D = CapsuleShape3D.new()
	capsuleShape.radius = capsuleData["radius"]
	# In Godot, it _appears_ the "height" of a capsule is the total end-to-end
	# distance [citation needed, seems undocumented], while in the glTF file,
	# the end-to-end distance is (radius + height + radius):
	capsuleShape.height = capsuleData["height"] + capsuleShape.radius * 2
	return capsuleShape

func makeCylinderShape(cylinderData : Dictionary) -> Shape3D:
	var cylinderShape : CylinderShape3D = CylinderShape3D.new()
	cylinderShape.height = cylinderData["height"]
	cylinderShape.radius = cylinderData["radius"]
	return cylinderShape

func makeConvexShape(state : GLTFState, convexData : Dictionary) -> Shape3D:
	var meshIdx : int = convexData["mesh"]
	var gltfMesh : GLTFMesh = state.get_meshes()[meshIdx]
	var importerMesh : ImporterMesh = gltfMesh.mesh
	var arrayMesh : ArrayMesh = importerMesh.get_mesh()
	var convexShape : ConvexPolygonShape3D = arrayMesh.create_convex_shape(true)
	return convexShape

func makeTriMeshShape(state : GLTFState, convexData : Dictionary) -> Shape3D:
	var meshIdx : int = convexData["mesh"]
	var gltfMesh : GLTFMesh = state.get_meshes()[meshIdx]
	var importerMesh : ImporterMesh = gltfMesh.mesh
	var arrayMesh : ArrayMesh = importerMesh.get_mesh()
	var concaveShape : ConcavePolygonShape3D = arrayMesh.create_trimesh_shape()
	return concaveShape

func _arrayToVector3(arr) -> Vector3:
	return Vector3(arr[0], arr[1], arr[2])

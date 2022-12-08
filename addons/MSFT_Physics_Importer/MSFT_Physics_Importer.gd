@tool
extends GLTFDocumentExtension
class_name MSFT_Physics

const extensionName : String = "MSFT_Physics" 
const c_ext : String = "extensions"

class PerDocumentPhysicsData:
	# Additional information per-GLTFDocument that we need to construct physics objects
	var nodeToGltfNodeMap : Dictionary

class PerNodePhysicsData:
	# Additional information per-GLTFNode that we need to construct physics objects
	var extensionData : Dictionary

func dumpTree(root : Node, indent=0):
	# todo.eoin REMOVE. For debugging.
	var tabs = "\t".repeat(indent)
	print(tabs, root.name, " (", root, ") owned by ", root.get_owner())
	for c in root.get_children():
		dumpTree(c, indent + 1)

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
	# Recursively traverse root and create rigid bodies and colliders
	var docData : PerDocumentPhysicsData = state.get_additional_data(extensionName)
	var newRoot : Node = _recurseCreateCollidersAndBodies(state, docData, root, false)
	_fixupNodeOwners(newRoot, root) # todo.eoin What if newRoot != root (i.e. root node has physics data somehow)?
	#dumpTree(newRoot) 
	return newRoot

func _recurseCreateCollidersAndBodies(state : GLTFState, docData : PerDocumentPhysicsData, curNode : Node3D, parentHasBody : bool) -> Node3D:
	# Need to unhook all the children, as we may need to be able to change their parent
	var originalChildren : Array[Node3D]
	originalChildren.append_array(curNode.get_children())
	for c in originalChildren:
		curNode.remove_child(c)
	
	var curNodeHasBody = false
	if docData.nodeToGltfNodeMap.has(curNode):
		var gltfNode : GLTFNode = docData.nodeToGltfNodeMap[curNode]
		curNodeHasBody = gltfNode.get_additional_data(extensionName) != null

	var newChildren : Array[Node3D]
	for c in originalChildren:
		newChildren.append(_recurseCreateCollidersAndBodies(state, docData, c, parentHasBody or curNodeHasBody))

	var outputNode : Node3D = curNode # We may need to add an additional parent to curNode

	if docData.nodeToGltfNodeMap.has(curNode):
		var gltfNode : GLTFNode = docData.nodeToGltfNodeMap[curNode]
		var perNodeData : PerNodePhysicsData = gltfNode.get_additional_data(extensionName)
		if perNodeData != null:
			var collisionShape : CollisionShape3D = null
			var rigidBody : RigidBody3D = null

			if perNodeData.extensionData.has("collider"):
				collisionShape = createCollider(state, perNodeData.extensionData["collider"])
			if perNodeData.extensionData.has("rigidBody"):
				rigidBody = createRigidBody(perNodeData.extensionData["rigidBody"])

			if rigidBody != null:
				rigidBody.name = str(curNode.name, "_rigidBody")
				outputNode = rigidBody
			elif collisionShape != null:
				# This node has a collision shape, but no rigid body
				if parentHasBody:
					# Just make a dummy container for holding the collider and the original hierarchy
					outputNode = Node3D.new()
					outputNode.name = str(curNode.name, "_colliderContainer")
				else:
					# We haven't seen a rigid body or collider in this branch before,
					# so make a static body, since collision shapes need to be parented to one
					outputNode = StaticBody3D.new()
					outputNode.name = str(curNode.name, "_staticBody")

			if collisionShape != null:
				outputNode.add_child(collisionShape)

			if outputNode != curNode:
				# Our new node needs to inherit the transform of the curNode,
				# and curNode should be a child with the identity transform
				outputNode.transform = curNode.transform
				outputNode.add_child(curNode)
				curNode.transform = Transform3D.IDENTITY

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

func _import_node(state : GLTFState, gltfNode : GLTFNode, jsonData : Dictionary, node : Node) -> int:
	var documentPhysics : PerDocumentPhysicsData = state.get_additional_data(extensionName)
	documentPhysics.nodeToGltfNodeMap[node] = gltfNode
	return OK

func createRigidBody(jsonData : Dictionary) -> RigidBody3D:
	var rigidBody : RigidBody3D = RigidBody3D.new()
	rigidBody.mass = jsonData["mass"]
	return rigidBody

func createCollider(state : GLTFState, jsonData : Dictionary) -> CollisionShape3D:	
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
	if jsonData.has("mesh"):
		return makeTriMeshShape(state, jsonData["mesh"])
	print_debug("Unhandled collider type", jsonData)
	return null

func makeSphereShape(sphereData : Dictionary) -> CollisionShape3D:
	var sphereShape : SphereShape3D = SphereShape3D.new()
	sphereShape.radius = sphereData["radius"]
	return makeCollisionShape(sphereShape)

func makeBoxShape(boxData : Dictionary) -> CollisionShape3D:
	var boxShape : BoxShape3D = BoxShape3D.new()
	boxShape.size = Vector3(toV3(boxData["size"]))
	return makeCollisionShape(boxShape)

func makeCapsuleShape(capsuleData : Dictionary) -> CollisionShape3D:
	var capsuleShape : CapsuleShape3D = CapsuleShape3D.new()
	capsuleShape.radius = capsuleData["radius"]
	# In Godot, it _appears_ the "height" of a capsule is the total end-to-end
	# distance [citation needed, seems undocumented], while in the glTF file,
	# the end-to-end distance is (radius + height + radius):
	capsuleShape.height = capsuleData["height"] + capsuleShape.radius * 2
	return makeCollisionShape(capsuleShape)

func makeCylinderShape(cylinderData : Dictionary) -> CollisionShape3D:
	var cylinderShape : CylinderShape3D = CylinderShape3D.new()
	cylinderShape.height = cylinderData["height"]
	cylinderShape.radius = cylinderData["radius"]
	return makeCollisionShape(cylinderShape)

func makeConvexShape(state : GLTFState, convexData : Dictionary) -> CollisionShape3D:
	var meshIdx : int = convexData["mesh"]
	var gltfMesh : GLTFMesh = state.get_meshes()[meshIdx]
	var importerMesh : ImporterMesh = gltfMesh.mesh
	var arrayMesh : ArrayMesh = importerMesh.get_mesh()
	var convexShape : ConvexPolygonShape3D = arrayMesh.create_convex_shape(true)
	return makeCollisionShape(convexShape)

func makeTriMeshShape(state : GLTFState, convexData : Dictionary) -> CollisionShape3D:
	var meshIdx : int = convexData["mesh"]
	var gltfMesh : GLTFMesh = state.get_meshes()[meshIdx]
	var importerMesh : ImporterMesh = gltfMesh.mesh
	var arrayMesh : ArrayMesh = importerMesh.get_mesh()
	var concaveShape : ConcavePolygonShape3D = arrayMesh.create_trimesh_shape()
	return makeCollisionShape(concaveShape)

func toV3(jsonData) -> Vector3:
	return Vector3(jsonData[0], jsonData[1], jsonData[2])

func makeCollisionShape(shape : Shape3D) -> CollisionShape3D:
	var collider = CollisionShape3D.new()
	collider.shape = shape
	collider.name = "Collider"
	return collider

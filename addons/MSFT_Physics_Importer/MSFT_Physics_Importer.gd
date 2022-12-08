@tool
extends GLTFDocumentExtension
class_name MSFT_Physics

const extensionName : String = "MSFT_Physics" 
const c_ext : String = "extensions"

class PerNodePhysicsData:
	var rbData : RigidBody3D = null
	var colliderData : CollisionShape3D = null
	var convexMeshIdx : int = -1 # Hack; meshes aren't created when we're creating convex shapes
	var concaveMeshIdx : int = -1 # Also hack

class PerDocumentPhysicsData:
	var nodeToGltfNodeMap : Dictionary

func dumpTree(root : Node, indent=0):
	# todo.eoin REMOVE. For debugging.
	var tabs = ""
	for i in range(indent):
		tabs = str("    ", tabs)
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

	var ret : Node3D = curNode

	if docData.nodeToGltfNodeMap.has(curNode):
		var gltfNode : GLTFNode = docData.nodeToGltfNodeMap[curNode]
		var physicsData : PerNodePhysicsData = gltfNode.get_additional_data(extensionName)

		if physicsData != null:	
			if physicsData.rbData == null:
				# Just make a dummy container for holding the collider and the original hierarchy
				var containerNode : Node3D = null
				if parentHasBody:
					containerNode = Node3D.new()
					containerNode.name = str(curNode.name, "_colliderContainer")
				else:
					# We haven't seen a rigid body or collider in this branch before,
					# so make a static body
					containerNode = StaticBody3D.new()
					containerNode.name = str(curNode.name, "_staticBody")
				ret = containerNode
			else:
				var rigidBody : RigidBody3D = physicsData.rbData
				rigidBody.name = str(curNode.name, "_rigidBody")
				ret = rigidBody

			if physicsData.colliderData != null:
				ret.add_child(physicsData.colliderData)
			elif physicsData.convexMeshIdx != -1:
				ret.add_child(_makeConvexShapeFromIndex(state, physicsData.convexMeshIdx))
			elif physicsData.concaveMeshIdx != -1:
				ret.add_child(_makeTriMeshShapeFromIndex(state, physicsData.concaveMeshIdx))

			if ret != curNode:
				# Our new node needs to inherit the transform of the curNode,
				# and curNode should be a child with the identity transform
				ret.transform = curNode.transform
				ret.add_child(curNode)
				curNode.transform = Transform3D.IDENTITY

	for c in newChildren:
		ret.add_child(c)
	return ret


func _get_supported_extensions():
	return [extensionName]

func _parse_node_extensions(state : GLTFState, gltfNode : GLTFNode, extensions : Dictionary):
	if not extensions.has(extensionName):
		return

	var physicsJson : Dictionary = extensions[extensionName]
	var physicsData : PerNodePhysicsData = PerNodePhysicsData.new()

	if physicsJson.has("collider"):
		# Hack! Shuffle code around so shape creation happens later, so it's cleaner
		var colliderData : Dictionary = physicsJson["collider"]
		if colliderData.has("convex"):
			physicsData.convexMeshIdx = colliderData["convex"]["mesh"]
		elif colliderData.has("mesh"):
			physicsData.concaveMeshIdx = colliderData["mesh"]["mesh"]
		else:
			physicsData.colliderData = createCollider(state, colliderData)
	if physicsJson.has("rigidBody"):
		physicsData.rbData = createRigidBody(physicsJson["rigidBody"])

	if (physicsData.rbData != null or physicsData.colliderData != null
		or physicsData.convexMeshIdx != -1 or physicsData.concaveMeshIdx != -1):
		gltfNode.set_additional_data(extensionName, physicsData)

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
	print("UNHANDLED COLLIDER TYPE: ", jsonData)
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
	capsuleShape.height = capsuleData["height"]
	capsuleShape.radius = capsuleData["radius"]
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

func _makeConvexShapeFromIndex(state : GLTFState, meshIdx : int) -> CollisionShape3D:
	var gltfMesh : GLTFMesh = state.get_meshes()[meshIdx]
	var importerMesh : ImporterMesh = gltfMesh.mesh
	var arrayMesh : ArrayMesh = importerMesh.get_mesh()
	var convexShape : ConvexPolygonShape3D = arrayMesh.create_convex_shape(true)
	return makeCollisionShape(convexShape)

func _makeTriMeshShapeFromIndex(state : GLTFState, meshIdx : int) -> CollisionShape3D:
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

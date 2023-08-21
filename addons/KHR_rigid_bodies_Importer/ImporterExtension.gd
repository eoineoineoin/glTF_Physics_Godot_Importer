@tool
extends EditorSceneFormatImporter

var khr_rigid_bodies = preload("res://addons/KHR_rigid_bodies_Importer/KHR_rigid_bodies_Importer.gd").new()

func _get_extensions():
	return ["gltf", "glb"]

func _get_import_flags():
	return IMPORT_SCENE

func _import_scene(path, flags, options):
	var doc = GLTFDocument.new()
	doc.register_gltf_document_extension(khr_rigid_bodies)
	var state : GLTFState = GLTFState.new()
	var docState = KHR_rigid_bodies.PerDocumentPhysicsData.new()
	state.set_additional_data(KHR_rigid_bodies.extensionName, docState)
	
	var err = doc.append_from_file(path, state, flags)
	if err != OK:
		return null

	var generated_scene = doc.generate_scene(state)
	return khr_rigid_bodies.postSceneConvert(state, generated_scene)

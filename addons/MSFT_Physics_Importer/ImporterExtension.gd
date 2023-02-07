@tool
extends EditorSceneFormatImporter

var msft_physics = preload("res://addons/MSFT_Physics_Importer/MSFT_Physics_Importer.gd").new()

func _get_extensions():
	return ["gltf", "glb"]

func _get_import_flags():
	return IMPORT_SCENE

func _import_scene(path, flags, options):
	var doc = GLTFDocument.new()
	doc.register_gltf_document_extension(msft_physics)
	var state : GLTFState = GLTFState.new()
	var docState = MSFT_Physics.PerDocumentPhysicsData.new()
	state.set_additional_data(MSFT_Physics.extensionName, docState)
	
	var err = doc.append_from_file(path, state, flags)
	if err != OK:
		return null

	var generated_scene = doc.generate_scene(state)
	return msft_physics.postSceneConvert(state, generated_scene)

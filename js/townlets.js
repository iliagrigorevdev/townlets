
const CAMERA_FOV = 13.0;
const CAMERA_NEAR = 1.0;
const CAMERA_FAR = 100.0;

const TOUCH_TO_SHOT_FACTOR = 0.2;
const SHOT_START_TOUCH_Y_MAX = -0.5;

var container, scene, camera, clock, renderer;
var batProtoMesh, townletProtoMesh, batMesh, townletMeshes;
var raycaster, pickTouchPoint, dragTouchPoint, shotPoint, dragVector, shotVector;
var startTime, dragging, shooting;
var physics;

function init() {
	scene = new THREE.Scene();
	//scene.background = new THREE.Color(0.7, 0.7, 0.7);

	camera = new THREE.PerspectiveCamera(CAMERA_FOV, window.innerWidth / window.innerHeight,
			CAMERA_NEAR, CAMERA_FAR);
	camera.position.set(0.0, 5.0, 12.0);
	camera.lookAt(new THREE.Vector3(0.0, 0.0, 2.0));

	var ambient = new THREE.AmbientLight(0xffffff, 0.4);
	scene.add(ambient);

	var light = new THREE.DirectionalLight(0xffffff, 0.6);
	light.position.set(0.4, 1.0, 0.6);
	light.castShadow = true;
	light.shadow.camera.left = -2.0;
	light.shadow.camera.right = 2.0;
	light.shadow.camera.top = 2.0;
	light.shadow.camera.bottom = -2.0;
	light.shadow.camera.near = 0.1;
	light.shadow.camera.far = 10.0;
	light.shadow.mapSize.width = 2048;
	light.shadow.mapSize.height = 2048;
	scene.add(light);

	var groundGeometry = new THREE.PlaneBufferGeometry(50.0, 50.0, 1, 1);
	var groundMesh = new THREE.Mesh(groundGeometry, new THREE.MeshPhongMaterial(
			{ color: new THREE.Color(0.7, 0.7, 0.7) }));
	groundMesh.receiveShadow = true;
	groundMesh.rotation.x = -Math.PI / 2.0;
	scene.add(groundMesh);

	raycaster = new THREE.Raycaster();
	pickTouchPoint = new THREE.Vector2();
	dragTouchPoint = new THREE.Vector2();
	shotPoint = new THREE.Vector3();
	dragVector = new THREE.Vector2();
	shotVector = new THREE.Vector3();
	startTime = 0;
	dragging = false;
	shooting = false;

	clock = new THREE.Clock();

	container = document.getElementById("container");

	renderer = new THREE.WebGLRenderer({ antialias: false });
	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setSize(window.innerWidth, window.innerHeight);
	renderer.gammaOutput = true;
	renderer.shadowMap.enabled = true;
	renderer.shadowMap.type = THREE.PCFSoftShadowMap;
	container.appendChild(renderer.domElement);

	window.addEventListener("resize", resizeViewport, false);

	var loader = new THREE.GLTFLoader();
	loader.load("res/townlets.gltf", (gltf) => {
		batProtoMesh = gltf.scene.children.find(child => child.name == "Bat");
		if (!batProtoMesh) {
			throw new Error("Bat mesh not found");
		}
		townletProtoMesh = gltf.scene.children.find(child => child.name == "Townlet");
		if (!townletProtoMesh) {
			throw new Error("Townlet mesh not found");
		}

		var maxAnisotropy = renderer.capabilities.getMaxAnisotropy();
		setAnisotropy(batProtoMesh, maxAnisotropy);
		setAnisotropy(townletProtoMesh, maxAnisotropy);

		Ammo().then((Ammo) => {
			initScene();
		});
	});
}

function setAnisotropy(parent, anisotropy) {
	parent.traverse((object) => {
		if (object.isMesh && object.material && object.material.map) {
			object.material.map.anisotropy = anisotropy;
		}
	});
}

function initScene() {
	physics = new Physics();

	createView();

	renderer.domElement.addEventListener("mousedown", onDocumentMouseDown, false);
	renderer.domElement.addEventListener("mousemove", onDocumentMouseMove, false);
	renderer.domElement.addEventListener("mouseup", onDocumentMouseUp, false);
	renderer.domElement.addEventListener("touchstart", onDocumentTouchStart, false);
	renderer.domElement.addEventListener("touchmove", onDocumentTouchMove, false);
	renderer.domElement.addEventListener("touchend", onDocumentTouchEnd, false);

	animate();
}

function updateScene(dt) {
	physics.update(dt);

	syncView();
}

function resizeViewport() {
	camera.aspect = window.innerWidth / window.innerHeight;
	camera.updateProjectionMatrix();
	renderer.setSize(window.innerWidth, window.innerHeight);
}

function createView() {
	if (!batMesh) {
		batMesh = batProtoMesh.clone();
		batMesh.castShadow = true;
		batMesh.receiveShadow = true;
		scene.add(batMesh);
	}

	if (!townletMeshes || (townletMeshes.length != physics.townletBodies.length)) {
		if (townletMeshes) {
			for (var i = 0; i < townletMeshes.length; i++) {
				scene.remove(townletMeshes[i]);
			}
		}
		townletMeshes = new Array(physics.townletBodies.length);
		for (var i = 0; i < townletMeshes.length; i++) {
			var townletMesh = townletProtoMesh.clone();
			townletMesh.castShadow = true;
			townletMesh.receiveShadow = true;
			scene.add(townletMesh);
			townletMeshes[i] = townletMesh;
		}
	}
}

function syncMeshToBody(mesh, body) {
	var transform = body.getCenterOfMassTransform();
	var p = transform.getOrigin();
	var q = transform.getRotation();
	mesh.position.set(p.x(), p.y(), p.z());
	mesh.quaternion.set(q.x(), q.y(), q.z(), q.w());
}

function syncView() {
	syncMeshToBody(batMesh, physics.batBody);
	for (var i = 0; i < townletMeshes.length; i++) {
		var townletMesh = townletMeshes[i];
		var townletBody = physics.townletBodies[i];
		syncMeshToBody(townletMesh, townletBody);
	}
}

function render() {
	var dt = clock.getDelta();

	updateScene(dt);

	renderer.render(scene, camera);
}

function animate() {
	requestAnimationFrame(animate);

	render();
}

function updateTouchRay(clientX, clientY, touchPoint) {
	var rect = renderer.domElement.getBoundingClientRect();
	touchPoint.x = ((clientX - rect.left) / rect.width) * 2.0 - 1.0;
	touchPoint.y = -((clientY - rect.top) / rect.height) * 2.0 + 1.0;
	raycaster.setFromCamera(touchPoint, camera);
}

function intersectTouchPlane(ray, point) {
	if (Math.abs(ray.direction.y) > 1e-5) { // ray direction must not be parallel to ground
		var t = -ray.origin.y / ray.direction.y;
		if (t >= 0.0) {
			point.copy(ray.direction).multiplyScalar(t).add(ray.origin);
			return true;
		}
	}
	return false;
}

function onActionDown(clientX, clientY, time) {
	dragging = false;
	shooting = false;

	updateTouchRay(clientX, clientY, pickTouchPoint);

	if (pickTouchPoint.y > SHOT_START_TOUCH_Y_MAX) {
		return;
	}

	if (!intersectTouchPlane(raycaster.ray, shotPoint)) {
		return;
	}

	dragging = true;
	startTime = time;
}

function onActionMove(clientX, clientY, time) {
	if (!dragging) {
		return;
	}

	updateTouchRay(clientX, clientY, dragTouchPoint);

	if (intersectTouchPlane(raycaster.ray, shotPoint)) {
		shooting = true;
	}
}

function onActionUp(clientX, clientY, time) {
	if (physics.simulationActive) {
		physics.reset(!dragging);
	} else if (shooting) {
		updateTouchRay(clientX, clientY, dragTouchPoint);

		intersectTouchPlane(raycaster.ray, shotPoint);

		dragVector.copy(dragTouchPoint).sub(pickTouchPoint);
		if (dragVector.y > 0.0) {
			var factor = (time > startTime)
					? TOUCH_TO_SHOT_FACTOR * dragVector.length() / (1e-3 * (time - startTime))
					: 1.0;
			var velocity = factor * SHOT_VELOCITY_MAX;
			if (velocity >= SHOT_VELOCITY_MIN) {
				var batPosition = physics.batBody.getCenterOfMassTransform().getOrigin();
				shotVector.set(-batPosition.x(),- batPosition.y(), -batPosition.z()).add(shotPoint);
				var angle = Math.atan2(-shotVector.x, -shotVector.z);
				var spin = SHOT_SPIN_MIN + (velocity - SHOT_VELOCITY_MIN) / (SHOT_VELOCITY_MAX - SHOT_VELOCITY_MIN)
						* (SHOT_SPIN_MAX - SHOT_SPIN_MIN);
				physics.shot(angle, velocity, spin);
			}
		}
	}

	dragging = false;
	shooting = false;
}

function onDocumentMouseDown(event) {
	event.preventDefault();

	onActionDown(event.clientX, event.clientY, event.timeStamp);
}

function onDocumentMouseMove(event) {
	event.preventDefault();

	onActionMove(event.clientX, event.clientY, event.timeStamp);
}

function onDocumentMouseUp(event) {
	event.preventDefault();

	onActionUp(event.clientX, event.clientY, event.timeStamp);
}

function onDocumentTouchStart(event) {
	var timeStamp = event.timeStamp;
	event.preventDefault();
	event = event.changedTouches[0];

	onActionDown(event.clientX, event.clientY, timeStamp);
}

function onDocumentTouchMove(event) {
	var timeStamp = event.timeStamp;
	event.preventDefault();
	event = event.changedTouches[0];

	onActionMove(event.clientX, event.clientY, timeStamp);
}

function onDocumentTouchEnd(event) {
	var timeStamp = event.timeStamp;
	event.preventDefault();
	event = event.changedTouches[0];

	onActionUp(event.clientX, event.clientY, timeStamp);
}

init();

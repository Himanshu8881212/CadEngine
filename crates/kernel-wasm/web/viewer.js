// Copyright (c) LMCAD. Licensed under the MIT License.

// Minimal Three.js harness: load the kernel WASM, mesh the demo model, and
// display it with orbit controls.
//
// The wasm package is produced by:  wasm-pack build --target web --out-dir web/pkg
// Serve this `web/` folder over HTTP (ES modules require http://, not file://).

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import init, { demo } from "./pkg/kernel_wasm.js";

const hud = document.getElementById("hud");

/** Build a Three.js BufferGeometry from the kernel's flattened MeshBuffers. */
function geometryFromBuffers(buffers) {
	const geometry = new THREE.BufferGeometry();
	geometry.setAttribute("position", new THREE.BufferAttribute(buffers.positions, 3));
	geometry.setAttribute("normal", new THREE.BufferAttribute(buffers.normals, 3));
	geometry.setIndex(new THREE.BufferAttribute(buffers.indices, 1));
	return geometry;
}

async function main() {
	// Instantiate the wasm module, then mesh the demo at a 0.5mm voxel size.
	await init();
	const buffers = demo(0.5);

	const canvas = document.getElementById("canvas");
	const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setSize(window.innerWidth, window.innerHeight);

	const scene = new THREE.Scene();
	scene.background = new THREE.Color(0x1e1e1e);

	const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
	camera.position.set(40, 30, 40);

	const controls = new OrbitControls(camera, renderer.domElement);
	controls.enableDamping = true;

	// Lighting.
	scene.add(new THREE.AmbientLight(0xffffff, 0.4));
	const key = new THREE.DirectionalLight(0xffffff, 0.9);
	key.position.set(30, 50, 20);
	scene.add(key);
	const fill = new THREE.DirectionalLight(0x8899ff, 0.4);
	fill.position.set(-30, -10, -20);
	scene.add(fill);

	// The kernel mesh.
	const geometry = geometryFromBuffers(buffers);
	const material = new THREE.MeshStandardMaterial({
		color: 0x4f9dff,
		metalness: 0.1,
		roughness: 0.6,
		flatShading: false,
	});
	const mesh = new THREE.Mesh(geometry, material);
	scene.add(mesh);

	// Frame the model and aim the controls at its centre.
	geometry.computeBoundingSphere();
	const sphere = geometry.boundingSphere;
	if (sphere) {
		controls.target.copy(sphere.center);
		const dist = sphere.radius * 3;
		camera.position.copy(sphere.center).add(new THREE.Vector3(dist, dist * 0.75, dist));
		camera.near = Math.max(0.01, sphere.radius * 0.01);
		camera.far = sphere.radius * 20;
		camera.updateProjectionMatrix();
	}
	controls.update();

	hud.textContent = `LMCAD kernel · ${buffers.triangleCount} triangles · ${buffers.vertexCount} vertices`;

	window.addEventListener("resize", () => {
		camera.aspect = window.innerWidth / window.innerHeight;
		camera.updateProjectionMatrix();
		renderer.setSize(window.innerWidth, window.innerHeight);
	});

	function animate() {
		requestAnimationFrame(animate);
		controls.update();
		renderer.render(scene, camera);
	}
	animate();
}

main().catch((err) => {
	hud.textContent = `LMCAD kernel · error: ${err}`;
	console.error(err);
});

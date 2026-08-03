// The 3D stage: three.js scene with neutral studio lighting, soft ground
// shadow, orbit controls, auto-framing on every new mesh, a top toolbar
// (view tools + the CLAUDE IS WORKING chip) and the view cube.
//
// Kernel STLs are Z-up (CAD convention): the camera's up axis is set to +Z so
// parts read the way the engine builds them, no geometry rotation involved.

import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'
import type { MeshReceipt } from '../api'
import { ViewCube } from './ViewCube'

interface Props {
	meshUrl: string | null
	working: boolean
	receipt: MeshReceipt | null
	onExport: () => void
	canExport: boolean
}

interface Rig {
	scene: THREE.Scene
	camera: THREE.PerspectiveCamera
	renderer: THREE.WebGLRenderer
	controls: OrbitControls
	key: THREE.DirectionalLight
	ground: THREE.Mesh
	part: THREE.Mesh | null
	radius: number
	center: THREE.Vector3
}

export function Viewport({ meshUrl, working, receipt, onExport, canExport }: Props) {
	const hostRef = useRef<HTMLDivElement>(null)
	const rigRef = useRef<Rig | null>(null)
	const cubeRef = useRef<HTMLDivElement>(null)
	const [wireframe, setWireframe] = useState(false)
	const [loading, setLoading] = useState(false)

	// Build the scene once.
	useEffect(() => {
		const host = hostRef.current!
		const scene = new THREE.Scene()
		scene.background = new THREE.Color('#f2f1ed')

		const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 10000)
		camera.up.set(0, 0, 1)
		camera.position.set(120, -120, 90)

		const renderer = new THREE.WebGLRenderer({ antialias: true })
		renderer.shadowMap.enabled = true
		renderer.shadowMap.type = THREE.PCFSoftShadowMap
		renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
		host.appendChild(renderer.domElement)

		const controls = new OrbitControls(camera, renderer.domElement)
		controls.enableDamping = true
		controls.dampingFactor = 0.12

		// Neutral studio lighting: soft sky dome + one shadowed key + cool fill.
		scene.add(new THREE.HemisphereLight(0xffffff, 0xb8b4ac, 0.9))
		const key = new THREE.DirectionalLight(0xffffff, 1.6)
		key.castShadow = true
		key.shadow.mapSize.set(2048, 2048)
		key.shadow.bias = -0.0004
		scene.add(key)
		const fill = new THREE.DirectionalLight(0xdfe6ee, 0.5)
		fill.position.set(-1.5, 2, 1.2)
		scene.add(fill)

		// Soft ground shadow plane (XY, Z-up world).
		const ground = new THREE.Mesh(
			new THREE.PlaneGeometry(1, 1),
			new THREE.ShadowMaterial({ opacity: 0.16 }),
		)
		ground.receiveShadow = true
		scene.add(ground)

		const rig: Rig = { scene, camera, renderer, controls, key, ground, part: null, radius: 60, center: new THREE.Vector3() }
		rigRef.current = rig

		const resize = () => {
			const w = host.clientWidth
			const h = host.clientHeight
			if (w === 0 || h === 0) return
			camera.aspect = w / h
			camera.updateProjectionMatrix()
			renderer.setSize(w, h)
		}
		resize()
		const observer = new ResizeObserver(resize)
		observer.observe(host)

		let raf = 0
		const tick = () => {
			raf = requestAnimationFrame(tick)
			controls.update()
			renderer.render(scene, camera)
			// Sync the CSS view cube to the camera orientation. CSS 3D is Y-down,
			// three.js is Y-up: negate the Y row of the view rotation (the same
			// conversion three's CSS3DRenderer applies to the camera matrix).
			const cube = cubeRef.current
			if (cube) {
				const m = new THREE.Matrix4().makeRotationFromQuaternion(camera.quaternion.clone().invert())
				const e = m.elements
				cube.style.transform = `matrix3d(${e[0]},${-e[1]},${e[2]},0,${e[4]},${-e[5]},${e[6]},0,${e[8]},${-e[9]},${e[10]},0,0,0,0,1)`
			}
		}
		tick()

		return () => {
			cancelAnimationFrame(raf)
			observer.disconnect()
			controls.dispose()
			renderer.dispose()
			host.removeChild(renderer.domElement)
			rigRef.current = null
		}
	}, [])

	// (Re)load the mesh whenever the URL changes.
	useEffect(() => {
		const rig = rigRef.current
		if (!rig || !meshUrl) return
		let cancelled = false
		setLoading(true)
		new STLLoader().load(
			meshUrl,
			(geometry) => {
				if (cancelled) return
				setLoading(false)
				if (rig.part) {
					rig.scene.remove(rig.part)
					rig.part.geometry.dispose()
				}
				const material = new THREE.MeshStandardMaterial({
					color: 0xc9c6bf,
					metalness: 0.15,
					roughness: 0.6,
					wireframe,
				})
				const mesh = new THREE.Mesh(geometry, material)
				mesh.castShadow = true
				rig.scene.add(mesh)
				rig.part = mesh
				frame(rig, geometry)
			},
			undefined,
			() => setLoading(false),
		)
		return () => {
			cancelled = true
		}
	}, [meshUrl]) // eslint-disable-line react-hooks/exhaustive-deps

	useEffect(() => {
		const part = rigRef.current?.part
		if (part) (part.material as THREE.MeshStandardMaterial).wireframe = wireframe
	}, [wireframe])

	const setView = (dir: [number, number, number]) => {
		const rig = rigRef.current
		if (!rig) return
		const d = new THREE.Vector3(...dir)
		// Looking straight down ±Z is degenerate with up = +Z; nudge off-axis.
		if (Math.abs(d.x) < 1e-6 && Math.abs(d.y) < 1e-6) d.y = -0.02 * Math.sign(d.z || 1)
		d.normalize()
		const dist = rig.radius * 2.8
		rig.camera.position.copy(rig.center.clone().add(d.multiplyScalar(dist)))
		rig.controls.target.copy(rig.center)
		rig.controls.update()
	}

	const fit = () => {
		const rig = rigRef.current
		if (rig?.part) frame(rig, rig.part.geometry)
	}

	return (
		<div className="stage" ref={hostRef}>
			{!meshUrl && <div className="vp-hint">NO GEOMETRY — chat a part into being, open a recipe, or insert from PARTS</div>}
			<div className="vp-toolbar">
				<button className="vp-btn" title="Undo (Wave 2)" disabled>↶</button>
				<button className="vp-btn" title="Redo (Wave 2)" disabled>↷</button>
				<span className="vp-sep" />
				<button className="vp-btn" onClick={fit} title="Fit view">FIT</button>
				<button className={wireframe ? 'vp-btn on' : 'vp-btn'} onClick={() => setWireframe((w) => !w)} title="Wireframe">WIRE</button>
				<button className="vp-btn" onClick={onExport} disabled={!canExport} title="Download the current STL">STL ↓</button>
				<span className="vp-sep" />
				{working ? (
					<span className="working-chip"><span className="dot" />CLAUDE IS WORKING</span>
				) : loading ? (
					<span className="idle-chip">LOADING MESH…</span>
				) : (
					<span className="idle-chip">IDLE</span>
				)}
			</div>
			<ViewCube cubeRef={cubeRef} onPick={setView} />
			{receipt && (
				<div className="receipt-strip">
					<span>vol {receipt.volume.toFixed(2)} mm³ ({receipt.volume_source})</span>
					<span className={receipt.route === 'exact' ? 'good' : 'warn'}>route {receipt.route}</span>
					<span>{receipt.tris.toLocaleString()} tris</span>
					<span className={receipt.watertight ? 'good' : 'warn'}>{receipt.watertight ? 'watertight' : 'NOT watertight'}</span>
				</div>
			)}
		</div>
	)
}

/** Frame the camera, ground plane and shadow frustum around `geometry`. */
function frame(rig: Rig, geometry: THREE.BufferGeometry) {
	geometry.computeBoundingBox()
	geometry.computeBoundingSphere()
	const box = geometry.boundingBox!
	const sphere = geometry.boundingSphere!
	const r = Math.max(sphere.radius, 1)
	rig.radius = r
	rig.center.copy(sphere.center)

	const dir = new THREE.Vector3(1, -1, 0.62).normalize()
	rig.camera.position.copy(sphere.center.clone().add(dir.multiplyScalar(r * 2.8)))
	rig.camera.near = r / 100
	rig.camera.far = r * 40
	rig.camera.updateProjectionMatrix()
	rig.controls.target.copy(sphere.center)
	rig.controls.update()

	rig.key.position.copy(sphere.center.clone().add(new THREE.Vector3(0.7, -0.5, 1.4).multiplyScalar(r * 2)))
	rig.key.target.position.copy(sphere.center)
	rig.key.target.updateMatrixWorld()
	const cam = rig.key.shadow.camera
	cam.left = -r * 1.6
	cam.right = r * 1.6
	cam.top = r * 1.6
	cam.bottom = -r * 1.6
	cam.near = r * 0.2
	cam.far = r * 6
	cam.updateProjectionMatrix()

	rig.ground.geometry.dispose()
	rig.ground.geometry = new THREE.PlaneGeometry(r * 14, r * 14)
	rig.ground.position.set(sphere.center.x, sphere.center.y, box.min.z - r * 0.002)
}

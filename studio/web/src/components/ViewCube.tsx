// The little orientation cube top-right of the viewport: a CSS-3D cube whose
// rotation is driven each frame from the camera quaternion (the Viewport owns
// that sync via `cubeRef`); clicking a face snaps the camera to that view.
// Z-up labels match the kernel's CAD convention.

import type { RefObject } from 'react'

interface Props {
	cubeRef: RefObject<HTMLDivElement | null>
	onPick: (dir: [number, number, number]) => void
}

const HALF = 24

const FACES: { label: string; dir: [number, number, number]; transform: string }[] = [
	{ label: 'FRONT', dir: [0, -1, 0], transform: `rotateX(90deg) translateZ(${HALF}px)` },
	{ label: 'BACK', dir: [0, 1, 0], transform: `rotateX(90deg) rotateY(180deg) translateZ(${HALF}px)` },
	{ label: 'RIGHT', dir: [1, 0, 0], transform: `rotateX(90deg) rotateY(90deg) translateZ(${HALF}px)` },
	{ label: 'LEFT', dir: [-1, 0, 0], transform: `rotateX(90deg) rotateY(-90deg) translateZ(${HALF}px)` },
	{ label: 'TOP', dir: [0, 0, 1], transform: `translateZ(${HALF}px)` },
	{ label: 'BOTTOM', dir: [0, 0, -1], transform: `rotateY(180deg) translateZ(${HALF}px)` },
]

export function ViewCube({ cubeRef, onPick }: Props) {
	return (
		<div className="viewcube">
			<div className="cube" ref={cubeRef}>
				{FACES.map((f) => (
					<button key={f.label} className="face" style={{ transform: f.transform }} onClick={() => onPick(f.dir)}>
						{f.label}
					</button>
				))}
			</div>
		</div>
	)
}

// PARAMS tab: the loaded part's Dims as numeric inputs + sliders. Committing a
// value (slider release / Enter / blur) fires one /api/part/set_dim call; the
// mesh, receipt and recipe file refresh from the kernel's answer.

import { useEffect, useState } from 'react'
import type { MeshReceipt, PartInfo } from '../api'

interface Props {
	part: PartInfo | null
	receipt: MeshReceipt | null
	onSetDim: (dim: string, value: number) => Promise<void>
	busy: boolean
}

/** Slider range around a value: 0.25×…4× its magnitude (sane for mm Dims). */
function rangeFor(value: number): { min: number; max: number; step: number } {
	const mag = Math.max(Math.abs(value), 1)
	return { min: Math.min(0, value), max: Math.ceil(mag * 4), step: mag >= 20 ? 1 : 0.1 }
}

function DimRow({ name, value, onCommit, busy }: { name: string; value: number; onCommit: (v: number) => void; busy: boolean }) {
	const [draft, setDraft] = useState(String(value))
	const [slide, setSlide] = useState(value)
	useEffect(() => {
		setDraft(String(value))
		setSlide(value)
	}, [value])
	const { min, max, step } = rangeFor(value)
	const commit = (v: number) => {
		if (Number.isFinite(v) && v !== value) onCommit(v)
	}
	return (
		<div className="param-row">
			<div className="param-head">
				<span className="param-name">{name}</span>
				<input
					className="param-value"
					value={draft}
					disabled={busy}
					onChange={(e) => setDraft(e.target.value)}
					onBlur={() => commit(parseFloat(draft))}
					onKeyDown={(e) => e.key === 'Enter' && commit(parseFloat(draft))}
				/>
			</div>
			<input
				type="range"
				min={min}
				max={max}
				step={step}
				value={slide}
				disabled={busy}
				onChange={(e) => {
					setSlide(parseFloat(e.target.value))
					setDraft(e.target.value)
				}}
				onMouseUp={() => commit(slide)}
				onTouchEnd={() => commit(slide)}
			/>
		</div>
	)
}

export function ParamsPanel({ part, receipt, onSetDim, busy }: Props) {
	if (!part) {
		return <div className="panel-body"><div className="empty-note">No recipe loaded — open one in the MODEL tab. Its named parameters (Dims) become live sliders here.</div></div>
	}
	return (
		<div className="panel-body">
			<div className="section-label">DIMS — {part.name}</div>
			{part.dims.length === 0 ? (
				<div className="empty-note">
					This recipe has no named parameters — every dimension is a literal in its feature tree
					(true of several gearbox parts, e.g. the shafts). Open a parametric one such as
					spacer_21 or key_2x2_8, or ask Claude to parameterize it.
				</div>
			) : (
				part.dims.map((d) => (
					<DimRow key={d.name} name={d.name} value={d.value} busy={busy} onCommit={(v) => void onSetDim(d.name, v)} />
				))
			)}
			{receipt && (
				<div className="receipt-card">
					<span className="k">VOLUME</span><span className="v">{receipt.volume.toFixed(3)} mm³ ({receipt.volume_source})</span>
					<span className="k">ROUTE</span><span className={receipt.route === 'exact' ? 'v good' : 'v warn'}>{receipt.route}</span>
					<span className="k">TRIS</span><span className="v">{receipt.tris.toLocaleString()}</span>
					<span className="k">WATERTIGHT</span><span className={receipt.watertight ? 'v good' : 'v warn'}>{String(receipt.watertight)}</span>
				</div>
			)}
		</div>
	)
}

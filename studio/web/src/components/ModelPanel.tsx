// MODEL tab: open a .lmcpart by repo-relative path (with quick links to the
// in-repo gearbox recipes), then the loaded part's header, BOM meta and
// feature history (kind + label, suppression shown honestly).

import { useState } from 'react'
import type { PartInfo } from '../api'

interface Props {
	part: PartInfo | null
	onOpen: (path: string) => void
	busy: boolean
}

const QUICK = [
	'gearbox/parts/spacer_21.lmcpart',
	'gearbox/parts/key_2x2_8.lmcpart',
	'gearbox/parts/shaft_input.lmcpart',
	'gearbox/parts/gear_s1_pinion.lmcpart',
	'gearbox/parts/housing_base.lmcpart',
]

export function ModelPanel({ part, onOpen, busy }: Props) {
	const [path, setPath] = useState('gearbox/parts/spacer_21.lmcpart')
	return (
		<div className="panel-body">
			<div className="section-label">OPEN RECIPE (.lmcpart)</div>
			<div className="open-row">
				<input
					className="mono"
					value={path}
					onChange={(e) => setPath(e.target.value)}
					onKeyDown={(e) => e.key === 'Enter' && onOpen(path)}
					placeholder="repo-relative path, e.g. gearbox/parts/spacer_21.lmcpart"
				/>
				<button className="tb-btn" onClick={() => onOpen(path)} disabled={busy}>OPEN</button>
			</div>
			<div className="quick">
				{QUICK.map((q) => (
					<button key={q} className="chip" onClick={() => { setPath(q); onOpen(q) }} disabled={busy}>
						{q.split('/').pop()}
					</button>
				))}
			</div>
			{part ? (
				<>
					<div className="section-label">PART</div>
					<div className="meta-grid">
						<span className="k">NAME</span><span>{part.name}</span>
						<span className="k">UNITS</span><span>{part.units}</span>
						<span className="k">CREATED WITH</span><span>{part.created_with || '—'}</span>
						{part.meta?.part_number != null && (<><span className="k">PART NO</span><span>{String(part.meta.part_number)}</span></>)}
						{part.active_config && (<><span className="k">ACTIVE CONFIG</span><span>{part.active_config}</span></>)}
						{Object.keys(part.configs ?? {}).length > 0 && (
							<><span className="k">CONFIGS</span><span>{Object.keys(part.configs).join(', ')}</span></>
						)}
					</div>
					<div className="section-label">FEATURES ({part.features.length})</div>
					{part.features.map((f) => (
						<div className="feature-row" key={f.index}>
							<span className="idx">{f.index}</span>
							<span className={f.suppressed ? 'kind suppressed' : 'kind'}>{f.kind}</span>
							{f.label && <span className="label">{f.label}</span>}
						</div>
					))}
				</>
			) : (
				<div className="empty-note">No recipe loaded. Open one above — its features, Dims and a live mesh will appear here, in PARAMS, and in the viewport.</div>
			)}
		</div>
	)
}

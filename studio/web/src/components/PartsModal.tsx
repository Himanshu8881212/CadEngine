// PARTS browser: the real /api/catalog families grouped by category; picking
// one shows its parameter schema pre-filled with the executed defaults, and
// INSERT submits an ordinary work order (part op + exact_volume + export_stl)
// to /api/run — the same path everything else uses.

import { useEffect, useMemo, useState } from 'react'
import type { Catalog, CatalogFamily, CatalogParam } from '../api'
import { getCatalog } from '../api'

interface Props {
	onClose: () => void
	onInsert: (program: unknown) => Promise<void>
}

function defaultValues(family: CatalogFamily): Record<string, string> {
	const out: Record<string, string> = {}
	for (const p of family.params) out[p.name] = String(p.default)
	return out
}

function parseValue(p: CatalogParam, raw: string): unknown {
	if (p.kind === 'bool') return raw === 'true'
	if (p.kind === 'string') return raw
	if (p.kind === 'int') return parseInt(raw, 10)
	return parseFloat(raw)
}

export function PartsModal({ onClose, onInsert }: Props) {
	const [catalog, setCatalog] = useState<Catalog | null>(null)
	const [error, setError] = useState<string | null>(null)
	const [selected, setSelected] = useState<CatalogFamily | null>(null)
	const [values, setValues] = useState<Record<string, string>>({})
	const [busy, setBusy] = useState(false)

	useEffect(() => {
		getCatalog()
			.then((c) => {
				setCatalog(c)
				if (c.families.length > 0) {
					setSelected(c.families[0])
					setValues(defaultValues(c.families[0]))
				}
			})
			.catch((e: Error) => setError(e.message))
	}, [])

	const grouped = useMemo(() => {
		const map = new Map<string, CatalogFamily[]>()
		for (const f of catalog?.families ?? []) {
			const list = map.get(f.category) ?? []
			list.push(f)
			map.set(f.category, list)
		}
		return [...map.entries()]
	}, [catalog])

	const pick = (f: CatalogFamily) => {
		setSelected(f)
		setValues(defaultValues(f))
	}

	const insert = async () => {
		if (!selected) return
		const op: Record<string, unknown> = { id: 'part', op: selected.op }
		for (const p of selected.params) {
			const v = parseValue(p, values[p.name] ?? String(p.default))
			if (v !== '' && !(typeof v === 'number' && Number.isNaN(v))) op[p.name] = v
		}
		const program = {
			ops: [
				op,
				{ id: 'v', op: 'exact_volume', in: 'part' },
				{ id: 'stl', op: 'export_stl', in: 'part', file: `${selected.op}.stl` },
			],
		}
		setBusy(true)
		try {
			await onInsert(program)
			onClose()
		} catch (e) {
			setError(e instanceof Error ? e.message : String(e))
		} finally {
			setBusy(false)
		}
	}

	return (
		<div className="modal-veil" onClick={onClose}>
			<div className="modal" onClick={(e) => e.stopPropagation()}>
				<div className="modal-head">
					<span>STANDARD PARTS CATALOG{catalog ? ` — ${catalog.count} FAMILIES` : ''}</span>
					<span className="grow" />
					<button className="tb-btn" onClick={onClose}>CLOSE</button>
				</div>
				<div className="modal-body">
					<div className="fam-list">
						{grouped.map(([category, families]) => (
							<div key={category}>
								<div className="fam-cat">{category.toUpperCase()}</div>
								{families.map((f) => (
									<button key={f.op} className={selected?.op === f.op ? 'fam-row sel' : 'fam-row'} onClick={() => pick(f)}>
										<div className="t">{f.title}</div>
										<div className="op">{f.op}</div>
									</button>
								))}
							</div>
						))}
					</div>
					<div className="fam-detail">
						{selected && (
							<>
								<h3>{selected.title}</h3>
								<div className="sum">{selected.summary}</div>
								{selected.params.map((p) => (
									<div className="fparam" key={p.name}>
										<span className="n">{p.name}{p.required ? '' : ' (opt)'}</span>
										{p.options ? (
											<select value={values[p.name] ?? ''} onChange={(e) => setValues({ ...values, [p.name]: e.target.value })}>
												{p.options.map((o) => (
													<option key={String(o)} value={String(o)}>{String(o)}</option>
												))}
											</select>
										) : p.kind === 'bool' ? (
											<select value={values[p.name] ?? 'false'} onChange={(e) => setValues({ ...values, [p.name]: e.target.value })}>
												<option value="false">false</option>
												<option value="true">true</option>
											</select>
										) : (
											<input type="text" className="mono" value={values[p.name] ?? ''} onChange={(e) => setValues({ ...values, [p.name]: e.target.value })} />
										)}
										<span className="m">{p.meaning}{p.min != null && p.max != null ? ` (${p.min}–${p.max})` : ''}</span>
									</div>
								))}
							</>
						)}
						{error && <div className="sum" style={{ color: 'var(--err)', marginTop: 12 }}>{error}</div>}
					</div>
				</div>
				<div className="modal-foot">
					<span className="note">INSERT runs: {selected ? `${selected.op} → exact_volume → export_stl` : '…'}</span>
					<button className="tb-btn primary" onClick={() => void insert()} disabled={!selected || busy}>
						{busy ? 'BUILDING…' : 'INSERT'}
					</button>
				</div>
			</div>
		</div>
	)
}

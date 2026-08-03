// The bottom-centre dark code pane: shows the model source being edited live —
// the last work order the AI ran, or the loaded .lmcpart envelope. Collapsible;
// lines that are new relative to the previously shown text get a diff-style
// `+` gutter so an edit reads as an edit.

import { useMemo, useState } from 'react'

interface Props {
	title: string
	text: string
	open: boolean
	onToggle: () => void
}

const MAX_LINES = 1600

export function CodePane({ title, text, open, onToggle }: Props) {
	// Derived-state-during-render (the supported React pattern): when the shown
	// text changes, remember the outgoing text as "previous" for the diff gutter.
	const [snapshot, setSnapshot] = useState<{ text: string; prev: string }>({ text: '', prev: '' })
	if (text !== snapshot.text) {
		setSnapshot({ text, prev: snapshot.text })
	}

	const lines = useMemo(() => {
		const next = snapshot.text.split('\n')
		const prevSet = new Set(snapshot.prev.split('\n'))
		const hadPrev = snapshot.prev.length > 0
		return next.slice(0, MAX_LINES).map((src, i) => ({
			n: i + 1,
			src,
			added: hadPrev && !prevSet.has(src) && src.trim() !== '',
		}))
	}, [snapshot])

	const total = snapshot.text === '' ? 0 : snapshot.text.split('\n').length

	return (
		<div className={open ? 'codepane' : 'codepane closed'}>
			<div className="code-head" onClick={onToggle} title={open ? 'Collapse' : 'Expand'}>
				<span>{open ? '▾' : '▸'}</span>
				<span>{title || 'MODEL SOURCE'}</span>
				<span className="lang">json</span>
				<span className="grow" />
				<span className="lang">{total} lines</span>
			</div>
			{open && (
				<div className="code-body">
					{lines.length === 0 ? (
						<div className="code-line"><span className="ln" /><span className="gut" /><span className="src" style={{ color: '#5d646f' }}>// nothing yet — run something</span></div>
					) : (
						lines.map((l) => (
							<div key={l.n} className={l.added ? 'code-line added' : 'code-line'}>
								<span className="ln">{l.n}</span>
								<span className="gut">{l.added ? '+' : ''}</span>
								<span className="src">{l.src}</span>
							</div>
						))
					)}
					{total > MAX_LINES && (
						<div className="code-line"><span className="ln" /><span className="gut" /><span className="src" style={{ color: '#5d646f' }}>… {total - MAX_LINES} more lines</span></div>
					)}
				</div>
			)}
		</div>
	)
}

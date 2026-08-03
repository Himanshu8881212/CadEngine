// CHAT tab: the conversation with the operator AI. Assistant turns render as
// a sequence of blocks — collapsible THINKING, streamed text, and tool-call
// status lines ("run_work_order · running" → "· done · 5 ops · ok").

import { useEffect, useRef, useState } from 'react'

export type AsstBlock =
	| { kind: 'text'; text: string }
	| { kind: 'thinking'; text: string }
	| { kind: 'tool'; name: string; ops: number; state: 'running' | 'done'; ok?: boolean; error?: string }

export type ChatItem = { role: 'user'; text: string } | { role: 'assistant'; blocks: AsstBlock[] }

interface Props {
	items: ChatItem[]
	working: boolean
	disabledNote: string | null
	onSend: (text: string) => void
}

function ToolLine({ b }: { b: Extract<AsstBlock, { kind: 'tool' }> }) {
	const dot = b.state === 'running' ? 'dot running' : b.ok ? 'dot ok' : 'dot err'
	return (
		<div className="toolline">
			<span className={dot} />
			<span>
				{b.name} · {b.state}
				{b.ops > 0 ? ` · ${b.ops} op${b.ops === 1 ? '' : 's'}` : ''}
				{b.state === 'done' ? ` · ${b.ok ? 'ok' : 'error'}` : ''}
			</span>
			{b.error && <span style={{ color: 'var(--err)' }}>{b.error}</span>}
		</div>
	)
}

export function ChatPanel({ items, working, disabledNote, onSend }: Props) {
	const [draft, setDraft] = useState('')
	const scrollRef = useRef<HTMLDivElement>(null)

	useEffect(() => {
		const el = scrollRef.current
		if (el) el.scrollTop = el.scrollHeight
	}, [items])

	const send = () => {
		const text = draft.trim()
		if (!text || working) return
		setDraft('')
		onSend(text)
	}

	return (
		<div className="chat">
			<div className="chat-scroll" ref={scrollRef}>
				{items.length === 0 && (
					<div className="chat-empty">
						Describe a part and the AI builds it with real kernel work orders — every claim
						receipted by the engine.
						<div className="hint">try: “make me a 30 mm cube with a Ø10 through-hole”</div>
						{disabledNote && <div className="hint" style={{ color: 'var(--err)' }}>{disabledNote}</div>}
					</div>
				)}
				{items.map((item, i) =>
					item.role === 'user' ? (
						<div className="msg-user" key={i}>{item.text}</div>
					) : (
						<div className="msg-asst" key={i}>
							{item.blocks.map((b, j) =>
								b.kind === 'text' ? (
									<div className="asst-text" key={j}>{b.text}</div>
								) : b.kind === 'thinking' ? (
									<details className="think" key={j}>
										<summary>THINKING</summary>
										<div className="think-body">{b.text}</div>
									</details>
								) : (
									<ToolLine b={b} key={j} />
								),
							)}
							{i === items.length - 1 && working && item.blocks.length === 0 && (
								<div className="toolline"><span className="dot running" /><span>thinking…</span></div>
							)}
						</div>
					),
				)}
			</div>
			<div className="chat-input">
				<textarea
					value={draft}
					placeholder={disabledNote ?? 'Describe the part…'}
					disabled={working || disabledNote != null}
					onChange={(e) => setDraft(e.target.value)}
					onKeyDown={(e) => {
						if (e.key === 'Enter' && !e.shiftKey) {
							e.preventDefault()
							send()
						}
					}}
				/>
				<button className="send-btn" onClick={send} disabled={working || disabledNote != null}>
					{working ? '…' : 'SEND'}
				</button>
			</div>
		</div>
	)
}

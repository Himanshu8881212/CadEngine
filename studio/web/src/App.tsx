// LMCAD Studio shell: chat left, viewport centre, code pane bottom-centre,
// parts catalog modal, params/model tabs. All geometry state flows one way:
// server receipt → React state → viewport/panels.

import { useRef, useState } from 'react'
import type { MeshReceipt, PartInfo, Report } from './api'
import { authorizationHeaders, chatStream, configureApiToken, loadPart, runProgram, setDim } from './api'
import type { AsstBlock, ChatItem } from './components/ChatPanel'
import { ChatPanel } from './components/ChatPanel'
import { CodePane } from './components/CodePane'
import { ModelPanel } from './components/ModelPanel'
import { ParamsPanel } from './components/ParamsPanel'
import { PartsModal } from './components/PartsModal'
import { TopBar } from './components/TopBar'
import { Viewport } from './components/Viewport'

type Tab = 'chat' | 'model' | 'params'

/** Pull a partial receipt out of a run report (volume / export measures). */
function receiptFromReport(report: Report): MeshReceipt | null {
	let volume: number | null = null
	let volumeSource = 'mesh'
	let route: string | null = null
	let tris = 0
	let watertight = false
	for (const op of report.ops) {
		const m = op.measures ?? {}
		if (typeof m['exact_volume'] === 'number') {
			volume = m['exact_volume']
			volumeSource = 'exact'
		} else if (volume === null && typeof m['volume'] === 'number') {
			volume = m['volume']
			volumeSource = 'mesh'
		}
		if (typeof m['route'] === 'string') {
			route = m['route']
			tris = typeof m['triangles'] === 'number' ? m['triangles'] : 0
			watertight = m['watertight'] === true
		}
	}
	if (route === null && volume === null) return null
	return {
		volume: volume ?? NaN,
		volume_source: volumeSource,
		route: route ?? '—',
		why: '',
		tris,
		watertight,
		artifact: { file: '', url: '', kind: 'stl' },
	}
}

let bust = 0

export default function App() {
	const [tab, setTab] = useState<Tab>('chat')
	const [part, setPart] = useState<PartInfo | null>(null)
	const [meshUrl, setMeshUrl] = useState<string | null>(null)
	const [receipt, setReceipt] = useState<MeshReceipt | null>(null)
	const [code, setCode] = useState<{ title: string; text: string }>({ title: 'MODEL SOURCE', text: '' })
	const [codeOpen, setCodeOpen] = useState(true)
	const [partsOpen, setPartsOpen] = useState(false)
	const [working, setWorking] = useState(false)
	const [busy, setBusy] = useState(false)
	const [chat, setChat] = useState<ChatItem[]>([])
	const [chatDisabled, setChatDisabled] = useState<string | null>(null)
	const [toast, setToast] = useState<string | null>(null)
	const turnsRef = useRef<{ role: 'user' | 'assistant'; content: string }[]>([])

	const fail = (e: unknown) => {
		setToast(e instanceof Error ? e.message : String(e))
		setTimeout(() => setToast(null), 8000)
	}

	const freshUrl = (url: string) => `${url}&v=${++bust}`
	const configureAuth = () => {
		const token = window.prompt('Studio API bearer token (blank clears it; stored only in this browser tab)')
		if (token === null) return
		configureApiToken(token)
		if (meshUrl) setMeshUrl(freshUrl(meshUrl.split('&v=')[0]))
		setToast(token.trim() ? 'API token configured for this tab' : 'API token cleared')
		setTimeout(() => setToast(null), 3000)
	}

	const openPart = async (path: string) => {
		setBusy(true)
		try {
			const info = await loadPart(path)
			setPart(info)
			setReceipt(info.receipt)
			setMeshUrl(freshUrl(info.receipt.artifact.url))
			setCode({ title: path, text: info.envelope })
			if (info.dims.length > 0) setTab('params')
			else setTab('model')
		} catch (e) {
			fail(e)
		} finally {
			setBusy(false)
		}
	}

	const changeDim = async (dim: string, value: number) => {
		if (!part) return
		setBusy(true)
		try {
			const resp = await setDim(part.path, dim, value)
			setPart({ ...part, dims: resp.dims, envelope: resp.envelope })
			setReceipt(resp.receipt)
			setMeshUrl(freshUrl(resp.receipt.artifact.url))
			setCode({ title: part.path, text: resp.envelope })
		} catch (e) {
			fail(e)
		} finally {
			setBusy(false)
		}
	}

	const insertCatalogPart = async (program: unknown) => {
		const resp = await runProgram(program)
		setCode({ title: 'work order — parts catalog', text: JSON.stringify(program, null, 2) })
		if (!resp.ok) {
			const firstError = resp.report.ops.find((o) => o.error)?.error
			throw new Error(firstError ? `${firstError.kind}: ${firstError.message}` : 'work order failed')
		}
		const stl = resp.artifacts.find((a) => a.kind === 'stl')
		if (stl) setMeshUrl(freshUrl(stl.url))
		const r = receiptFromReport(resp.report)
		if (r) setReceipt(r)
	}

	const exportStl = async () => {
		if (!meshUrl) return
		const response = await fetch(meshUrl, { headers: authorizationHeaders() })
		if (!response.ok) throw new Error(`mesh download failed: ${response.status}`)
		const objectUrl = URL.createObjectURL(await response.blob())
		const a = document.createElement('a')
		a.href = objectUrl
		a.download = (receipt?.artifact.file || part?.name || 'part') + (receipt?.artifact.file ? '' : '.stl')
		document.body.appendChild(a)
		a.click()
		a.remove()
		URL.revokeObjectURL(objectUrl)
	}

	const sendChat = async (text: string) => {
		turnsRef.current.push({ role: 'user', content: text })
		setChat((items) => [...items, { role: 'user', text }, { role: 'assistant', blocks: [] }])
		setWorking(true)
		// Blocks accumulate in a local array; every change flushes an immutable
		// snapshot into the LAST chat item (StrictMode-safe: the updater only
		// replaces, never mutates).
		const blocks: AsstBlock[] = []
		const flush = () =>
			setChat((items) => [...items.slice(0, -1), { role: 'assistant', blocks: blocks.map((b) => ({ ...b })) }])
		let assistantText = ''
		try {
			for await (const ev of chatStream(turnsRef.current)) {
				if (ev.type === 'text') {
					assistantText += ev.delta
					const tail = blocks[blocks.length - 1]
					if (tail?.kind === 'text') tail.text += ev.delta
					else blocks.push({ kind: 'text', text: ev.delta })
					flush()
				} else if (ev.type === 'thinking') {
					const tail = blocks[blocks.length - 1]
					if (tail?.kind === 'thinking') tail.text += ev.delta
					else blocks.push({ kind: 'thinking', text: ev.delta })
					flush()
				} else if (ev.type === 'tool') {
					if (ev.state === 'running') {
						if (ev.program !== undefined) {
							setCode({ title: 'work order — claude', text: JSON.stringify(ev.program, null, 2) })
						}
						blocks.push({ kind: 'tool', name: ev.name, ops: ev.ops, state: 'running' })
					} else {
						const tool = [...blocks].reverse().find((b) => b.kind === 'tool' && b.state === 'running')
						if (tool && tool.kind === 'tool') {
							tool.state = 'done'
							tool.ops = ev.ops
							tool.ok = ev.ok
							tool.error = ev.error
						}
					}
					flush()
				} else if (ev.type === 'refresh') {
					const stl = ev.artifacts.find((a) => a.kind === 'stl')
					if (stl) setMeshUrl(freshUrl(stl.url))
					if (ev.receipt) setReceipt(ev.receipt)
				} else if (ev.type === 'chat_disabled') {
					setChatDisabled(ev.message)
					blocks.push({ kind: 'text', text: ev.message })
					flush()
				} else if (ev.type === 'error') {
					blocks.push({ kind: 'text', text: `error: ${ev.message}` })
					flush()
				}
			}
		} catch (e) {
			fail(e)
		} finally {
			setWorking(false)
			turnsRef.current.push({ role: 'assistant', content: assistantText || '(tool activity only)' })
		}
	}

	const docTitle = part ? `${part.path}` : 'untitled session'

	return (
		<div className="app">
			<TopBar
				docTitle={docTitle}
				onParts={() => setPartsOpen(true)}
				onToggleCode={() => setCodeOpen((o) => !o)}
				codeOpen={codeOpen}
				onExport={() => void exportStl().catch(fail)}
				onAuth={configureAuth}
				canExport={meshUrl != null}
			/>
			<aside className="left">
				<div className="tabs">
					<button className={tab === 'chat' ? 'tab active' : 'tab'} onClick={() => setTab('chat')}>CHAT</button>
					<button className={tab === 'model' ? 'tab active' : 'tab'} onClick={() => setTab('model')}>MODEL</button>
					<button className={tab === 'params' ? 'tab active' : 'tab'} onClick={() => setTab('params')}>
						PARAMS{part && part.dims.length > 0 && <span className="badge">{part.dims.length}</span>}
					</button>
				</div>
				{tab === 'chat' && <ChatPanel items={chat} working={working} disabledNote={chatDisabled} onSend={(t) => void sendChat(t)} />}
				{tab === 'model' && <ModelPanel part={part} onOpen={(p) => void openPart(p)} busy={busy} />}
				{tab === 'params' && <ParamsPanel part={part} receipt={receipt} onSetDim={changeDim} busy={busy} />}
			</aside>
			<main className="stage-wrap" style={{ position: 'relative', minWidth: 0, minHeight: 0 }}>
				<Viewport meshUrl={meshUrl} working={working} receipt={receipt} onExport={() => void exportStl().catch(fail)} canExport={meshUrl != null} />
				<CodePane title={code.title} text={code.text} open={codeOpen} onToggle={() => setCodeOpen((o) => !o)} />
			</main>
			{partsOpen && <PartsModal onClose={() => setPartsOpen(false)} onInsert={insertCatalogPart} />}
			{toast && <div className="err-toast">{toast}</div>}
		</div>
	)
}

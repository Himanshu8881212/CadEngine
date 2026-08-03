// LMCAD Studio API client — thin typed wrappers over the studio-server
// endpoints. Every geometry number shown in the UI comes from these payloads
// (kernel receipts), never from client-side computation.

export interface Artifact {
	file: string
	url: string
	kind: string
}

export interface OpReport {
	id: string
	ok: boolean
	measures?: Record<string, unknown>
	file?: string
	error?: { kind: string; message: string }
}

export interface Report {
	ok: boolean
	ops: OpReport[]
}

export interface RunResponse {
	ok: boolean
	session: string
	report: Report
	artifacts: Artifact[]
}

export interface DimInfo {
	name: string
	value: number
}

export interface FeatureInfo {
	index: number
	kind: string
	label?: string
	suppressed: boolean
}

export interface MeshReceipt {
	volume: number
	volume_source: string
	route: string
	why: string
	tris: number
	watertight: boolean
	artifact: Artifact
}

export interface PartInfo {
	path: string
	name: string
	units: string
	created_with: string
	meta?: Record<string, unknown>
	dims: DimInfo[]
	features: FeatureInfo[]
	configs: Record<string, Record<string, number>>
	active_config?: string
	envelope: string
	receipt: MeshReceipt
}

export interface SetDimResponse {
	ok: boolean
	dim: string
	before: number
	after: number
	volume_before: number
	dims: DimInfo[]
	receipt: MeshReceipt
	envelope: string
}

export interface CatalogParam {
	name: string
	kind: 'number' | 'int' | 'bool' | 'string'
	required: boolean
	default: unknown
	min?: number
	max?: number
	options?: unknown[]
	meaning: string
}

export interface CatalogFamily {
	op: string
	title: string
	category: string
	summary: string
	params: CatalogParam[]
}

export interface Catalog {
	count: number
	families: CatalogFamily[]
}

const SESSION = 'default'

async function post<T>(url: string, body: unknown): Promise<T> {
	const resp = await fetch(url, {
		method: 'POST',
		headers: { 'content-type': 'application/json' },
		body: JSON.stringify(body),
	})
	const data = await resp.json()
	if (!resp.ok) throw new Error((data as { error?: string }).error ?? `${url} failed: ${resp.status}`)
	return data as T
}

export function runProgram(program: unknown): Promise<RunResponse> {
	return post<RunResponse>('/api/run', { program, session: SESSION })
}

export function loadPart(path: string): Promise<PartInfo> {
	return post<PartInfo>('/api/part/load', { path, session: SESSION })
}

export function setDim(path: string, dim: string, value: number): Promise<SetDimResponse> {
	return post<SetDimResponse>('/api/part/set_dim', { path, dim, value, session: SESSION })
}

export function savePart(path: string, envelope: unknown): Promise<{ ok: boolean; bytes: number }> {
	return post('/api/part/save', { path, envelope })
}

export async function getCatalog(): Promise<Catalog> {
	const resp = await fetch('/api/catalog')
	if (!resp.ok) throw new Error(`catalog failed: ${resp.status}`)
	return (await resp.json()) as Catalog
}

// ---- Chat (SSE over fetch; EventSource cannot POST) -------------------------

export type ChatEvent =
	| { type: 'text'; delta: string }
	| { type: 'thinking'; delta: string }
	| { type: 'tool'; state: 'running' | 'done'; name: string; ops: number; ok?: boolean; error?: string; program?: unknown }
	| { type: 'refresh'; artifacts: Artifact[]; receipt?: MeshReceipt }
	| { type: 'chat_disabled'; message: string }
	| { type: 'error'; message: string }
	| { type: 'done'; stop_reason: string }

export interface ChatTurn {
	role: 'user' | 'assistant'
	content: string
}

/** POST /api/chat and yield parsed SSE events until the stream closes. */
export async function* chatStream(messages: ChatTurn[]): AsyncGenerator<ChatEvent> {
	const resp = await fetch('/api/chat', {
		method: 'POST',
		headers: { 'content-type': 'application/json' },
		body: JSON.stringify({ messages, session: SESSION }),
	})
	if (!resp.ok || !resp.body) {
		const text = await resp.text().catch(() => '')
		throw new Error(`chat failed: ${resp.status} ${text}`)
	}
	const reader = resp.body.getReader()
	const decoder = new TextDecoder()
	let buffer = ''
	for (;;) {
		const { done, value } = await reader.read()
		if (done) break
		buffer += decoder.decode(value, { stream: true })
		for (;;) {
			const cut = buffer.indexOf('\n\n')
			if (cut < 0) break
			const frame = buffer.slice(0, cut)
			buffer = buffer.slice(cut + 2)
			let event = 'message'
			const dataLines: string[] = []
			for (const line of frame.split('\n')) {
				if (line.startsWith('event:')) event = line.slice(6).trim()
				else if (line.startsWith('data:')) dataLines.push(line.slice(5).trimStart())
			}
			if (dataLines.length === 0) continue
			try {
				const data = JSON.parse(dataLines.join('\n')) as Record<string, unknown>
				yield { type: event, ...data } as ChatEvent
			} catch {
				// keep-alive comments / non-JSON frames are ignored
			}
		}
	}
}

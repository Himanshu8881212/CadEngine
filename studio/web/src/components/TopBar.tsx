// Title bar: brand + current document, then PARTS / CODE / IMPORT / EXPORT /
// SHARE. IMPORT and SHARE are visible but land in Wave 2.

interface Props {
	docTitle: string
	onParts: () => void
	onToggleCode: () => void
	codeOpen: boolean
	onExport: () => void
	canExport: boolean
}

export function TopBar({ docTitle, onParts, onToggleCode, codeOpen, onExport, canExport }: Props) {
	return (
		<header className="topbar">
			<span className="brand">LMCAD <em>STUDIO</em></span>
			<span className="doc-title">{docTitle}</span>
			<span className="spacer" />
			<button className="tb-btn" onClick={onParts}>PARTS</button>
			<button className={codeOpen ? 'tb-btn primary' : 'tb-btn'} onClick={onToggleCode}>CODE</button>
			<button className="tb-btn" disabled title="Wave 2">IMPORT</button>
			<button className="tb-btn" onClick={onExport} disabled={!canExport} title="Download the current STL">EXPORT</button>
			<button className="tb-btn" disabled title="Wave 2">SHARE</button>
		</header>
	)
}

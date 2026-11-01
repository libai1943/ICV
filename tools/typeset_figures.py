"""Typeset every editable subfigure at 9.5 printed points with compact panel rows.

Geometry remains vector content copied from the original figure. Use --refresh
with a figure stem after regenerating that numerical figure from new results.
"""
from pathlib import Path
import sys,re,json,shutil,hashlib
import argparse,os
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
parser.add_argument('--font',type=Path,default=Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts/times.ttf')
parser.add_argument('--italic-font',type=Path,default=None)
args=parser.parse_args();C=args.root.resolve();T=C/'.figure-layout';T.mkdir(exist_ok=True)
import pymupdf as f
fontfile=str(args.font);font=f.Font(fontfile=fontfile);italicfile=str(args.italic_font or args.font.with_name('timesi.ttf'));italic=f.Font(fontfile=italicfile);PRINT_WIDTH=120*72/25.4;PRINT_HEIGHT=191*72/25.4;SIZE=9.5
active={};refresh=set();previous={}
manifest=T/'hashes.json'
if manifest.exists():previous=json.loads(manifest.read_text())
for p in C.rglob('fig*.pdf'):
 if 'output' not in p.relative_to(C).parts:continue
 fraction={ 'fig01-03':.94, 'fig04-05':.94 }.get(p.stem,1)
 active[p]=(PRINT_WIDTH*fraction,PRINT_HEIGHT*.78 if p.stem=='fig01-03' else None)
 if hashlib.sha256(p.read_bytes()).hexdigest()!=previous.get(str(p.relative_to(C))):refresh.add(p.stem)
audit=[]
for file,(printW,printH) in active.items():
 if not file.exists():continue
 backup=T/'subcaption-originals'/file.name;backup.parent.mkdir(exist_ok=True)
 if not backup.exists() or file.stem in refresh:shutil.copy2(file,backup)
 doc=f.open(backup);p=doc[0];W=p.rect.width;initial_scale=printW/W;labels=[];graphics=[]
 for b in p.get_text('dict')['blocks']:
  for l in b.get('lines',[]):
   s=''.join(z['text'] for z in l['spans']).strip().replace('\xa0',' ');r=f.Rect(l['bbox'])
   if re.match(r'^\([a-h]\)(?:\s|$)',s):labels.append((r,s))
   else:graphics.append(r)
 if not labels:doc.close();continue
 for d in p.get_drawings():
  if d['rect'].width>W*.95 and d['rect'].height>p.rect.height*.95:continue
  if d.get('fill') and min(d['fill'])>.98 and not d.get('color'):continue
  graphics.append(d['rect'])
 for im in p.get_image_info():graphics.append(f.Rect(im['bbox']))
 rows=[]
 for r,s in sorted(labels,key=lambda v:v[0].y0):
  if not rows or abs(r.y0-rows[-1][0])>10/initial_scale:rows.append([r.y0,[]])
  rows[-1][1].append((r,s))
 plans=[];last=0
 for y,group in rows:
  group.sort(key=lambda v:v[0].x0);centers=[(r.x0+r.x1)/2 for r,s in group];panels=[]
  for i,((r,s),cx) in enumerate(zip(group,centers)):
   x0=(centers[i-1]+cx)/2 if i else 0;x1=(cx+centers[i+1])/2 if i+1<len(group) else W
   region=f.Rect(x0,last,x1,y-.5/initial_scale);rects=[q&region for q in graphics if q.intersects(region)]
   if not rects:raise ValueError((file.name,s,'no panel content'))
   bottom=max(q.y1 for q in rects);top=min(q.y0 for q in rects);clip=f.Rect(x0,max(0,top-.3/initial_scale),x1,bottom+.3/initial_scale)
   panels.append((clip,s,cx))
  plans.append((max(clip.height for clip,s,cx in panels),panels));last=max(r.y1 for r,s in group)+.5/initial_scale
 footer_rects=[q for q in graphics if last<=q.y0<p.rect.height and q.y1<=p.rect.height+30/initial_scale];footer=None
 if footer_rects:footer=f.Rect(0,min(q.y0 for q in footer_rects),W,min(p.rect.height,max(q.y1 for q in footer_rects)+1/initial_scale))
 source_height=sum(h for h,_ in plans)+(footer.height if footer else 0)
 capheight=(font.ascender-font.descender)*SIZE;fixed=2+len(plans)*(5+capheight)+10*(len(plans)-1)+(10 if footer else 1)+2
 scale=initial_scale
 if printH:scale=min(scale,(printH-fixed)/source_height)
 assert scale>0
 height=source_height*scale+fixed;width=W*scale;new=f.open();page=new.new_page(width=width,height=height);page.insert_font(fontname='BookSubcaption',fontfile=fontfile);page.insert_font(fontname='BookTime',fontfile=italicfile);cursor=2
 for row_index,(h,panels) in enumerate(plans):
  baseline=cursor+h*scale+5+font.ascender*SIZE
  for clip,s,cx in panels:
   dest=f.Rect(clip.x0*scale,cursor+(h-clip.height)*scale,clip.x1*scale,cursor+h*scale);page.show_pdf_page(dest,doc,0,clip=clip,keep_proportion=False)
   parts=re.split(r'(?<= )([tT])(?=\s*=)',s);runs=[(z,'BookTime' if i%2 else 'BookSubcaption',italic if i%2 else font) for i,z in enumerate(parts)];tw=sum(ft.text_length(z,fontsize=SIZE) for z,nm,ft in runs);x=cx*scale-tw/2
   for z,nm,ft in runs:
    page.insert_text((x,baseline),z,fontname=nm,fontsize=SIZE);x+=ft.text_length(z,fontsize=SIZE)
   audit.append({'file':file.name,'text':s,'printed_font_pt':SIZE,'panel_gap_pt':5,'baseline':baseline,'center_x':cx*scale})
  cursor=baseline-font.descender*SIZE+(10 if row_index+1<len(plans) or footer else 1)
 if footer:
  page.show_pdf_page(f.Rect(0,cursor,width,cursor+footer.height*scale),doc,0,clip=footer,keep_proportion=False);cursor+=footer.height*scale
 # Keep exact height when a LaTeX height cap determines scaling; otherwise trim.
 if not printH or scale==initial_scale:page.set_cropbox(f.Rect(0,0,page.rect.width,min(page.rect.height,cursor+2)))
 out=file.with_suffix('.typeset.pdf');new.save(out,garbage=4,deflate=True);new.close();doc.close();out.replace(file)
(T/'subcaption-normalization.json').write_text(json.dumps(audit,indent=2),encoding='utf8')
print(json.dumps({'figures':len(set(x['file'] for x in audit)),'subcaptions':len(audit),'printed_font_pt':SIZE,'panel_gap_pt':5}))

manifest.write_text(json.dumps({str(p.relative_to(C)):hashlib.sha256(p.read_bytes()).hexdigest() for p in active},indent=2))

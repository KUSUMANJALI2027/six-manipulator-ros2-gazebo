"""Place cropped screenshot viewports in a labeled PDF figure; originals unchanged."""
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
from PIL import Image
root=Path(__file__).resolve().parents[1]
panels=[('SCARA_workspace_rviz.png',(535,235,1018,575),'(a) SCARA: RViz workspace'),('Stanford_rviz.png',(371,275,660,540),'(b) Stanford-type: RViz'),('PUMA_rviz.png',(371,245,660,555),'(c) PUMA-type: RViz'),('SCORBOT_rviz.png',(880,335,1140,530),'(d) SCORBOT-type: RViz'),('Delta_gazebo.png',(210,205,870,495),'(e) Delta: Gazebo'),('Stewart_gazebo.png',(235,205,700,675),'(f) Stewart: Gazebo')]
c=canvas.Canvas(str(root/'figures/six_robot_overview.pdf'),pagesize=(504,288))
for i,(name,(l,t,r,b),label) in enumerate(panels):
 x=(i%3)*170;y=288-(i//3+1)*144;w=164;h=120
 iw,ih=Image.open(root/'figures'/name).size
 scale=min(w/(r-l),h/(b-t));dw=(r-l)*scale;dh=(b-t)*scale
 cx=x+(w-dw)/2;cy=y+22+(h-dh)/2
 c.saveState();p=c.beginPath();p.rect(cx,cy,dw,dh);c.clipPath(p,stroke=0)
 c.drawImage(str(root/'figures'/name),cx-l*scale,cy-(ih-b)*scale,width=iw*scale,height=ih*scale)
 c.restoreState();c.setFont('Times-Roman',9);c.drawCentredString(x+w/2,y+8,label)
c.save()

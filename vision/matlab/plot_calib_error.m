function plot_calib_error
	D = importdata('cam-calib-error.txt');
	Ex = reshape(D.data(:,3),11,8)';
	Ey = reshape(D.data(:,4),11,8)';
	E = sqrt((Ex.^2)+(Ey.^2));
	figure;
	surf([0:100:1000],[0:100:700],Ex,abs(Ex),'FaceColor','interp');
	axis([0,1000,0,700,-5,5]);
	caxis([0,5]);
	figure;
	surf([0:100:1000],[0:100:700],Ey,abs(Ey),'FaceColor','interp');
	axis([0,1000,0,700,-5,5]);
	caxis([0,5]);
	figure;
	surf([0:100:1000],[0:100:700],E,'FaceColor','interp');
	axis([0,1000,0,700,0,5]);
	caxis([0,5]);
	
	figure; hold;
	axis([-40,1040,-40,740]);
	for x=0:100:1000,
		plot([x,x],[0,700],'k-');
	end
	for y=0:100:700,
		plot([0,1000],[y,y],'k-');
	end
	for x=0:100:1000,
		for y=0:100:700,
			plot([x+Ex(round(y/100)+1,round(x/100)+1)],[y+Ey(round(y/100)+1,round(x/100)+1)],'ok');
		end
	end
	

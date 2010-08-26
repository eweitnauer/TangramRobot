function [fig1, fig2] = plot_calib_error
	D = importdata('cam-calib-error.txt');
	Ex = reshape(D.data(:,3),11,8)';
	Ey = reshape(D.data(:,4),11,8)';
	E = sqrt((Ex.^2)+(Ey.^2));
	fig1 = figure();
	surf([0:100:1000],[0:100:700],E,'FaceColor','interp');
	set(gca,'YLim',[0,700],'YTick',[0:200:700],'YTickMode','manual','XLim',[0,1000],'XTick',[0:200:1000],'XTickMode','manual','ZLim',[0,5],'ZTick',[0:5],'ZTickMode','manual','FontSize',8);
	cb = colorbar;
	caxis([0,5]);
	xlabel('x in mm','FontSize',8);
	ylabel('y in mm','FontSize',8);
	zlabel('error in mm','FontSize',8);
	pbaspect manual;
	pbaspect([1 0.7 0.6]);
	set(fig1,'PaperUnits', 'centimeters', 'PaperSize', [7,6], 'PaperPositionMode','manual', 'PaperPosition', [0,0,7,6]);
	set(cb,'Position',[0.9 0.05 0.04 0.9],'FontSize',8);
	set(gca,'Position',[0.13 0.05 0.63 0.9]);
		
	fig2 = figure; hold;
	xlabel('x in mm','FontSize',8);
	ylabel('y in mm','FontSize',8);
	set(gca,'YLim',[0,700],'YTick',[0:200:700],'YTickMode','manual','XLim',[0,1000],'XTick',[0:200:1000],'XTickMode','manual','FontSize',8);
	axis([-40,1040,-40,740]);
	axis equal;
	for x=0:100:1000,
		plot([x,x],[0,700],'k-','LineWidth',0.4);
	end
	for y=0:100:700,
		plot([0,1000],[y,y],'k-','LineWidth',0.4);
	end
	for x=0:100:1000,
		for y=0:100:700,
			plot([x+Ex(round(y/100)+1,round(x/100)+1)],[y+Ey(round(y/100)+1,round(x/100)+1)],'+b','LineWidth',0.8);
		end
	end
	set(fig2, 'PaperUnits', 'centimeters', 'PaperSize', [7 6], 'PaperPosition', [0,0,7,6]);


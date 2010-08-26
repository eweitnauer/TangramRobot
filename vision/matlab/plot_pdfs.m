function plot_pdfs
	[f1,f2] = plot4pdf;
	print(f1,'-dpdf','calibration_error_surf.pdf');
	print(f2,'-dpdf','calibration_error_plot.pdf');
	close(f1);
	close(f2);

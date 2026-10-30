function BookExportTables(book,folder)
%BOOKEXPORTTABLES Common SI-unit trajectory table for all four applications.
if ~isfolder(folder),mkdir(folder);end
rows=[];
for i=1:size(book.state,3)
 n=numel(book.time);rows=[rows;[repmat(i,n,1),book.time(:),book.state(:,:,i)',book.control(:,:,i)']]; %#ok<AGROW>
end
names={'vehicle','time_s','x_m','y_m','theta_rad','v_m_s','phi_rad','a_m_s2','omega_rad_s'};
writetable(array2table(rows,'VariableNames',names),fullfile(folder,'trajectory.csv'));
end

function BookPlotStyle(figures)
%BOOKPLOTSTYLE Shared typography and export conventions for all book examples.
set(groot,'defaultAxesFontName','Times New Roman', ...
    'defaultTextFontName','Times New Roman', ...
    'defaultLegendFontName','Times New Roman');
if nargin==0, return; end
for fig=figures(:)'
    set(fig,'Color','w');
    objects=findall(fig,'-property','FontName');
    set(objects,'FontName','Times New Roman');
    axesList=findall(fig,'Type','axes');
    set(axesList,'FontSize',11,'LineWidth',0.7,'TickDir','out', ...
        'GridAlpha',0.12,'XColor','k','YColor','k');
    texts=findall(fig,'Type','text');
    for k=1:numel(texts)
        set(texts(k),'BackgroundColor','none');
    end
end
end

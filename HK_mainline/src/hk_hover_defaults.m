function p=hk_hover_defaults()
s=hk_hover_schema();p=struct();
for k=1:numel(s),p.(s(k).key)=s(k).default;end
end

function v = IS_MX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 108);
  end
  v = vInitialized;
end

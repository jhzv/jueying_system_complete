function h(g, d) {
  var a = new Image();
  a.onload = function() {
    var r = document.createElement("canvas"), t = r.getContext("2d");
    if (!t)
      throw new Error("Failed to create Canvas context!");
    r.width = a.width, r.height = a.height, t.imageSmoothingEnabled = !1, t.drawImage(a, 0, 0);
    for (var n = t.getImageData(0, 0, a.width, a.height).data, o = "", e = 0; e < n.length; e += 4)
      o += String.fromCharCode(
        n[e],
        n[e + 1],
        n[e + 2]
      );
    d(JSON.parse(o));
  }, a.src = "data:image/png;base64," + g;
}
export {
  h as default
};

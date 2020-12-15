import numpy as np

TANGO_HTML_COLORS = {"butter1": "#fce94f",
                     "butter2": "#edd400",
                     "butter3": "#c4a000",
                     "orange1": "#fcaf3e",
                     "orange2": "#f57900",
                     "orange3": "#ce5c00",
                     "chocolate1": "#e9b96e",
                     "chocolate2": "#c17d11",
                     "chocolate3": "#8f5902",
                     "chameleon1": "#8ae234",
                     "chameleon2": "#73d216",
                     "chameleon3": "#4e9a06",
                     "skyblue1": "#729fcf",
                     "skyblue2": "#3465a4",
                     "skyblue3": "#204a87",
                     "plum1": "#ad7fa8",
                     "plum2": "#75507b",
                     "plum3": "#5c3566",
                     "scarletred1": "#ef2929",
                     "scarletred2": "#cc0000",
                     "scarletred3": "#a40000",
                     "aluminium1": "#eeeeec",
                     "aluminium2": "#d3d7cf",
                     "aluminium3": "#babdb6",
                     "aluminium4": "#888a85",
                     "aluminium5": "#555753",
                     "aluminium6": "#2e3436",
                     "white": "#ffffff",
                     "black": "#000000"}

HEX = {k: int.from_bytes(bytes.fromhex(v[1:]), 'big') for k, v in TANGO_HTML_COLORS.items()}
NP4 = {k: np.array([(v >> 16) & 255, (v >> 8) & 255, (v >> 0) & 255, 255]).astype('f4')/255. for k, v in HEX.items()}
NP = {k: v[:3] for k, v in NP4.items()}
for k, v in NP.items():
    vars()[k] = v

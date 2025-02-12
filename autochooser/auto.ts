type Pose = { x: number; y: number; heading: number };
type Color = { r: number; g: number; b: number };
// Ghost option: [destination label, pose, trajectory file path]
type GhostOption = [string, Pose, string];

enum points {
    st, // Start Top
    sm, // Start Middle
    sb, // Start Bottom
    rtr, // Reef Top Right
    rr, // Reef Right
    rbr, // Reef Buttom Right
    rtl, // Reef Top left
    rl, // Reef Left
    rbl, // Reef Bottom left
    p, // Processor
    ht, // Human Player Station Top
    hb // Human Player Station Bottom
}

enum actions {
    l1, // Level 1
    l4l, // Level 4 Left
    l4r, // Level 4 Right
    dealgify, // De-Algify
    processor, // Processor
    hpl, // Human Player Station Left
    hpm,
    hpr
}

var headings = {
    'st': 180,
    'sm': 180,
    'sb': 180,
    'rtr': 240,
    'rr': 180,
    'rbr': 120,
    'rbl': 60,
    'rl': 0,
    'rtl': 300,
    'p': 270,
    'ht': 306,
    'hb': 54
}

var field: HTMLElement;
var startDots: HTMLCollectionOf<Element> = [] as unknown as HTMLCollectionOf<Element>;
var startPose: Pose | null = null;

document.onload = () => {
    field = document.getElementById("fieldsvg")!;
    startDots = document.getElementsByClassName("start")!;
    for (let i = 0; i < startDots.length; i++) {
        startDots[i].addEventListener('mousedown', () => {
            setStart({
                x: parseInt(startDots[i].getAttribute("cx")!),
                y: parseInt(startDots[i].getAttribute("cy")!),
                heading: headings[startDots[i].id as keyof typeof headings]
            });
        });
    }
}

function setStart(pose: Pose) {

}

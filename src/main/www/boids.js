const SLOW_REFRESH_DELAY = 1000 / 3;
const FAST_REFRESH_DELAY = 1000 / 30;
let REFRESH_DELAY = FAST_REFRESH_DELAY;

ABORT_TIMEOUT = () => ({ signal: AbortSignal.timeout(500) })

// Color of the background
const BACKGROUND_COLOR = "#FAFAFA"
// Color of the reference solution boids
const REF_COLOR = "#8AE234AA"
// Default color of the boids if their color is not sent in the request
const DEFAULT_BOID_COLOR = "#0000FF"
// The base size of a boid on screen (which is then scaled by a scale factor)
const BOID_SIZE = 3

// from https://stackoverflow.com/a/1909508/1846915
function delay(fn, ms) {
  let timer = 0
  return function(...args) {
    clearTimeout(timer)
    timer = setTimeout(fn.bind(this, ...args), ms || 0)
  }
}

const VERBOSE = false;
const verbose = VERBOSE ? console.debug : () => {};

class Canvas {
    MAX_ZOOM_LEVEL = 20;
    PRIMARY_BUTTON_MASK = 1;

    constructor(root) {
        this.root = root;

        this.model = {};
        this.trace = false;
        this.resetTrace = false;

        // root.width, root.height is resolution (logical size),
        // clientWidth, clientHeight is physical size on page.
        this.root.width = this.root.clientWidth;
        this.root.height = this.root.clientHeight;

        this.ctx = this.root.getContext("2d");
        this.ctx.globalCompositeOperation = "source-over";

        this.root.onwheel = (event) => this.onWheel(event);
        this.root.onmousemove = (event) => this.onMouseMove(event);

        this.setLimits({ xmin: -this.root.width / 2, xmax: this.root.width / 2,
                         ymin: -this.root.height / 2, ymax: this.root.height / 2 });
        this.resetBoidsViewport();
    }

    setLimits(limits) {
        this.limits = limits;
        this.resetTranslation();
        return this;
    }

    resetTranslation() {
        this.resetTrace = true;
        this.origin = { x: this.root.width / 2, y: this.root.height / 2 };
        if (this.limits) {
            const {xmin, xmax, ymin, ymax } = this.limits;
            this.translateOrigin(- (xmax + xmin) / 2, - (ymax + ymin) / 2);
        }
    }

    setZoomLevel(level) {
        this.resetTrace = true;
        this.zoomLevel = level;
        this.scale = Math.pow(1.1, level);
        return this;
    }

    translateOrigin(deltaX, deltaY) {
        this.resetTrace = true;
        this.origin = { x: this.origin.x + deltaX, y: this.origin.y + deltaY };
        return this;
    }

    setModel(model) {
        this.model = model;
        return this;
    }

    onWheel(event) {
        event.preventDefault();

        const oldScale = this.scale;

        const lvl = this.zoomLevel + (event.deltaY < 0 ? 1 : -1);
        this.setZoomLevel(Math.max(-this.MAX_ZOOM_LEVEL, Math.min(this.MAX_ZOOM_LEVEL, lvl)));

        const bbox = this.root.getBoundingClientRect();
        const mX = event.clientX - bbox.x;
        const mY = event.clientY - bbox.y;
        const zoomRatio = (oldScale - this.scale) / oldScale;
        this.translateOrigin(zoomRatio * (mX - this.origin.x), zoomRatio * (mY - this.origin.y));

        this.render();
    }

    onMouseMove(event) {
        event.preventDefault();
        if (event.buttons & this.PRIMARY_BUTTON_MASK)
            this.translateOrigin(event.movementX, event.movementY).render();
    }

    resetBoidsViewport() {
        this.resetTrace = true;
        this.setZoomLevel(-2);
        this.resetTranslation();
        return this;
    }

    clear() {
        this.ctx.clearRect(0, 0, this.root.width, this.root.height);
        this.renderLimits();
    }

    transform(point) {
        return { x: this.origin.x + point.x * this.scale,
                 y: this.origin.y + point.y * this.scale };
    }

    renderLimits() {
        this.ctx.fillStyle = BACKGROUND_COLOR;
        this.ctx.beginPath();
        this.ctx.rect(0, 0, this.root.width, this.root.height);
        if (this.limits !== undefined) {
          this.ctx.rect( // Negative width means exclude this area
              this.origin.x + this.limits.xmax * this.scale,
              this.origin.y + this.limits.ymin * this.scale,
              (this.limits.xmin - this.limits.xmax) * this.scale,
              (this.limits.ymax - this.limits.ymin) * this.scale
          )
        }
        this.ctx.fill();
    }

    renderBoid = function(boid, isRef) {
        const { x, y } = this.transform(boid);
        const norm = Math.sqrt(boid.vx * boid.vx + boid.vy * boid.vy);
        const scale = boid.size ?? 1.0;
        let style = boid.rgb ?? DEFAULT_BOID_COLOR;

        if (isRef) {
            style = REF_COLOR
            this.ctx.beginPath()
            this.ctx.arc(x, y, BOID_SIZE * 4, 0, 2 * Math.PI, false);
            this.ctx.lineWidth = BOID_SIZE / 2;
            this.ctx.strokeStyle = REF_COLOR;
            this.ctx.stroke();
        }

        this.ctx.fillStyle = style;
        this.ctx.beginPath();
        if (norm) {
            const vx = boid.vx / norm * scale * BOID_SIZE;
            const vy = boid.vy / norm * scale * BOID_SIZE;
            // Center of boid is at (x, y)
            this.ctx.moveTo(x + 2 * vx, y + 2 * vy);
            this.ctx.lineTo((x - 2 * vx) - vy, (y -  2 * vy) + vx);
            this.ctx.lineTo((x - 2 * vx) + vy, (y - 2 * vy) - vx);
        } else {
            this.ctx.rect(x - 2, y - 2, 4, 4);
        }
        this.ctx.fill();
    }

    renderFlock(flock, isRef) {
        flock.forEach((boid) => { 
            this.renderBoid(boid, isRef);
        });
    }

    render() {
        if (!this.trace)
            this.clear();
        else if (this.resetTrace) {
            this.clear();
            this.resetTrace = false;
        }
        if (this.model.flock)
            this.renderFlock(this.model.flock, false);
        if (this.model.reference)
            this.renderFlock(this.model.reference, true);
    }
}

class App {
    DEFAUlT_RANDOM_BOIDS_COUNT = 500;
    MAX_RANDOM_BOIDS_COUNT = 10000

    initConfigBox = document.getElementById("initConfig");

    limitsInputs = Array.from(document.querySelectorAll("#limits input"));
    physicsInputs = Array.from(document.querySelectorAll("#physics input"));
    textboxes = [...this.limitsInputs, ...this.physicsInputs];
    boidsCountElem = document.getElementById("boidsCount");
    boidsCountInput = document.querySelector("#boidsCount input");

    resetViewportButton = document.getElementById("resetViewButton");
    runButton = document.getElementById("runButton");
    stepButton = document.getElementById("debugStep");
    resetButton = document.getElementById("resetButton");
    presetsSelect = document.getElementById("presets");
    referenceCheckbox = document.getElementById("reference");
    canvas = document.getElementById("canvas");

    running = false;
    fetching = false;
    boidJson = null;
    lastFetchDate = 0;
    stepIndex = 0;
    lastRandomBoidsCount = this.DEFAUlT_RANDOM_BOIDS_COUNT;

    textboxesEditable = true;
    viewActive = false;
    initial = undefined;
    reference = undefined;

    serverDown = function(err) {
        console.debug(err);
        this.stop();
        document
            .getElementById("serverDown")
            .style.visibility = "visible";
    }

    uiRunning = function() {
        this.running = true;
        runButton.textContent = "pause";
    }

    uiNotRunning = function() {
        this.running = false;
        runButton.textContent = "run";
    }

    activateView = function() {
        this.viewActive = true;
        this.runButton.disabled = false;
        this.stepButton.disabled = false;
        this.canvas.style.background = "white";
    }

    deactivateView = function() {
        this.viewActive = false;
        this.runButton.disabled = true;
        this.stepButton.disabled = true;
        this.canvas.style.background = "#b0b0b0";
    }

    activateTextboxes = function() {
        this.initConfigBox.disabled = false;
        this.boidsCountInput.disabled = false;
        this.textboxes.forEach((tb) => tb.disabled = false);
    }

    deactivateTextboxes = function() {
        this.initConfigBox.disabled = true;
        this.boidsCountInput.disabled = true;
        this.textboxes.forEach((tb) => tb.disabled = true);
    }

    activateReference = function(r) {
        this.reference = r;
        this.referenceCheckbox.checked = true;
        this.referenceCheckbox.disabled = false;
    }

    deactivateReference = function() {
        this.reference = undefined;
        this.referenceCheckbox.checked = false;
        this.referenceCheckbox.disabled = true;
    }

    runningState = function() {
        this.uiRunning();
        this.deactivateTextboxes();
        this.activateView();
        if (this.initial == undefined) console.debug("initial undefined while running!");
    }

    steppingState = function() {
        this.uiNotRunning();
        this.deactivateTextboxes();
        this.activateView();
        if (this.initial == undefined) console.debug("initial undefined while stepping!");
    }

    presetReadyState = function() {
        this.uiNotRunning();
        this.activateTextboxes();
        this.activateView();
        if (this.initial == undefined) console.debug("initial undefined while preset ready!");
        if (this.reference == undefined) console.debug("reference undefined while preset ready!");
    }

    nonPresetReadyState = function() {
        this.uiNotRunning();
        this.activateTextboxes();
        this.activateView();
        if (this.initial == undefined) console.debug("initial undefined while non-preset ready!");
        this.deactivateReference();
    }

    nonReadyState = function() {
        this.uiNotRunning();
        this.activateTextboxes();
        this.deactivateView();
        this.initial = undefined;
        this.deactivateReference();
    }

    run = function() {
        if (!this.viewActive) console.debug("should not be able to click run while view not active!");
        this.runningState();
        this.loop(this.lastFetchDate);
    }

    step = function() {
        if (!this.viewActive) console.debug("should not be able to click step while view not active!");
        this.steppingState();
        this.fetchUpdateAndRender(true);
    }

    stop = function() {
        if (!this.viewActive) console.debug("should not be able to click stop while view not active!");
        this.steppingState();
        this.displayBoids();
    }

    reset = function() {
        this.initializeWith(this.initial);
        if (this.reference == undefined) {
            this.nonPresetReadyState();
        } else {
            this.presetReadyState();
        }
    }

    initializeWith = function(config) {
        let {reference, ...config_data} = config
        return fetch('http://localhost:8080/initializeWith', {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(config_data),
            ...ABORT_TIMEOUT()
        })
            .then((response) => response.json())
            .then((config) => {
                this.initial = config;
                this.stepIndex = 0;
                this.boidJson = config.initialBoids;
                this.display
                    .resetBoidsViewport()
                    .setLimits(config.physics.limits)
                    .clear();
                this.displayBoids();
            })
            .catch((err) => this.serverDown(err));
    }

    handleTestCaseResponse = function(request) {
        return request
            .then((response) => response.json())
            .then((testCase) => {
                this.activateReference(testCase.reference || []);
                this.initConfigBox.value = JSON.stringify(testCase.initialBoids);
                const { limits, ...physics } = testCase.physics;
                for (const obj of [limits, physics]) {
                  for (const [param, value] of Object.entries(obj)) {
                      document.getElementById(param).value = value.toString();
                  }
                }
                return testCase;
            })
            .then((testCase) => {
                this.initializeWith(testCase)
                    .then(() => this.presetReadyState());
            })
            .catch((err) => this.serverDown(err));
    }

    initializeRandomRequest = function(boidsCount) { return fetch(`http://localhost:8080/initializeRandom`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(
                {
                    boidsCount: boidsCount,
                    physics: 
                    {
                        limits: this.inputsToObject(this.limitsInputs),
                        ...this.inputsToObject(this.physicsInputs)
                    }
                }
            ),
            ...ABORT_TIMEOUT()
        });
    }

    testCaseRequest = function(preset) { 
        return fetch(`http://localhost:8080/testCase/${preset}.json`, ABORT_TIMEOUT()); 
    }

    selectPreset = function(preset) {
        return this.handleTestCaseResponse(preset == "random" ? this.initializeRandomRequest(this.lastRandomBoidsCount) : this.testCaseRequest(preset))
    }

    inputsToObject = function(inputs) {
        const target = {};
        for (const tb of inputs) {
            var parsed = parseFloat(tb.value);
            if (!Number.isFinite(parsed)) {
                throw new TypeError();
            } else {
                target[tb.id] = parsed;
            }
        }
        return target;
    }

    tryConfiguration = function() {
        try {
            return this.initializeWith({
              initialBoids: JSON.parse(this.initConfigBox.value),
              physics: {
                  limits: this.inputsToObject(this.limitsInputs),
                  ...this.inputsToObject(this.physicsInputs)
              }
            });
        } catch (e) {
            return false;
        }
    }

    changeTextboxes = function() {
        this.presetsSelect.value = "custom";
        this.nonReadyState();
        var attempt = this.tryConfiguration();
        if (attempt) {
            attempt.then(() => this.nonPresetReadyState());
        }
    }

    constructor(display) {
        this.display = display;
        this.resetViewportButton.addEventListener("click", (event) => {
            event.preventDefault();
            this.display.resetBoidsViewport().render();
        });
        this.runButton.addEventListener("click", () => {
            if (this.running) {
                this.stop();
            } else {
                this.run();
            }
        });
        this.boidsCountElem.style.visibility="hidden";
        this.boidsCountInput.addEventListener("input", (e) => {
            if (e.target.value > 0 && e.target.value <= this.MAX_RANDOM_BOIDS_COUNT && e.target.value != this.lastRandomBoidsCount) {
                this.lastRandomBoidsCount = e.target.value;
                this.handleTestCaseResponse(this.initializeRandomRequest(this.lastRandomBoidsCount));
            }
        })
        this.stepButton.addEventListener("click", () => this.step());
        this.resetButton.addEventListener("click", () => this.reset());
        this.presetsSelect.addEventListener("change", (event) => {
            this.boidsCountElem.style.visibility = event.target.value == "random" ? "visible" : "hidden";
            this.boidsCountInput.value = this.lastRandomBoidsCount;
            if (event.target.value == "custom") {
                this.initializeWith(this.initial);
                this.nonPresetReadyState();
            } else {
                this.selectPreset(event.target.value);
            }
        });
        this.initConfigBox.addEventListener(
            "input",
            delay(() => this.changeTextboxes(), 200));
        for (const tb of this.textboxes) {
            tb.addEventListener("input", delay(() => this.changeTextboxes(), 200));
        }
        fetch('http://localhost:8080/testCases', ABORT_TIMEOUT())
            .then((response) => response.json())
            .then((testCases) => {
                for (const testCase of testCases) {
                    var opt = document.createElement('option');
                    opt.value = testCase;
                    opt.innerHTML = testCase.slice(3);
                    this.presetsSelect.appendChild(opt);
                }
                var random = document.createElement('option');
                random.value = "random";
                random.innerHTML = "random"
                this.presetsSelect.appendChild(random);
                var custom = document.createElement('option');
                custom.value = "custom";
                custom.innerHTML = "write your own…"
                this.presetsSelect.appendChild(custom);
                this.selectPreset("00_singleBoidNoForces");
            })
            .catch((err) => this.serverDown(err));
    }

    displayBoids = function() {
        this.display.setModel({
            flock: this.boidJson,
            reference: this.reference !== undefined &&
                this.referenceCheckbox.checked &&
                this.stepIndex < this.reference.length &&
                this.reference[this.stepIndex]
        }).render();
    }

    fetchUpdateAndRender = function(forceRender) {
        if (this.fetching) {
            verbose("Skipped frame");
            this.late = true;
            return;
        }
        this.fetching = true;
        var oldStepIndex = this.stepIndex;
        return fetch(`http://localhost:8080/get`, ABORT_TIMEOUT())
            .then((response) => response.json())
            .then((boidJson) => {
                if (this.stepIndex == oldStepIndex) {
                    verbose(`Update at ${this.stepIndex} took ${boidJson.elapsedMs}ms`)
                    this.stepIndex += 1;
                    this.boidJson = boidJson.boids;
                } else {
                    console.debug("outdated response");
                }
            })
            .catch((err) => this.serverDown(err))
            .finally(() => {
                this.fetching = false;
                if (this.late || forceRender || !this.running) {
                    this.displayBoids();
                    this.late = false;
                }
            });
    }

    loop(time_elapsed) {
        if (!this.running) {
            return;
        }

        window.requestAnimationFrame((new_time) => this.loop(new_time));
        this.displayBoids();

        // If we hit the refresh rate
        if(time_elapsed - this.lastFetchDate >= REFRESH_DELAY) {
            verbose(`Refreshing after ${(time_elapsed - this.lastFetchDate)}`);
            this.lastFetchDate = time_elapsed;
            this.fetchUpdateAndRender(false);
        }
    }
}

function updateTrace(canvas) {
    canvas.trace = trace.checked;
}

function updateSlow() {
    REFRESH_DELAY = slow.checked ? SLOW_REFRESH_DELAY : FAST_REFRESH_DELAY;
}

function Run() {
    const canvas = new Canvas(document.getElementById("canvas"));

    const trace = document.getElementById("trace");
    trace.addEventListener("change", () => updateTrace(canvas));
    updateTrace(canvas);

    const slow = document.getElementById("slow");
    slow.addEventListener("change", updateSlow);
    updateSlow();

    window.boidsApp = new App(canvas);

    const legendFlock = [{"x": -3, "y": 0, "vx": 1, "vy": 0}];
    new Canvas(document.getElementById("legendMine"))
        .setModel({ flock: legendFlock }).render();
    new Canvas(document.getElementById("legendReference"))
        .setModel({ reference: legendFlock }).render();
    new Canvas(document.getElementById("legendOutside"))
        .setLimits(undefined).render();
}

window.onload = () => Run();

import SwiftUI
import MetalKit
import simd

struct MetalMeterLevel: Identifiable, Equatable {
    var id: UInt8
    var label: String
    var rawLabel: String
    var normalized: Float
    var colorFamily: Float
}

struct MetalMeterBankView: View {
    var levels: [MetalMeterLevel]
    var height: CGFloat = 72

    var body: some View {
        VStack(alignment: .leading, spacing: 7) {
            MetalMeterStrip(levels: levels)
                .frame(height: height)
                .background(Color.secondary.opacity(0.08))
                .clipShape(RoundedRectangle(cornerRadius: 8))

            LazyVGrid(columns: [GridItem(.adaptive(minimum: 64), spacing: 8)], spacing: 5) {
                ForEach(levels) { level in
                    VStack(alignment: .leading, spacing: 2) {
                        Text(level.label)
                            .font(.caption2.bold())
                            .lineLimit(1)
                            .minimumScaleFactor(0.75)
                        Text(level.rawLabel)
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundStyle(.secondary)
                            .lineLimit(1)
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                }
            }
        }
    }
}

private struct MetalMeterStrip: NSViewRepresentable {
    var levels: [MetalMeterLevel]

    func makeNSView(context: Context) -> MetalMeterHostView {
        let view = MetalMeterHostView()
        view.update(levels: levels)
        return view
    }

    func updateNSView(_ nsView: MetalMeterHostView, context: Context) {
        nsView.update(levels: levels)
    }
}

private final class MetalMeterHostView: NSView {
    private var metalView: MTKView?
    private var renderer: MetalMeterRenderer?
    private var fallbackLabel: NSTextField?

    override init(frame frameRect: NSRect) {
        super.init(frame: frameRect)
        wantsLayer = true

        guard let device = MTLCreateSystemDefaultDevice() else {
            installFallbackLabel("Metal unavailable")
            return
        }

        let view = MTKView(frame: .zero, device: device)
        view.translatesAutoresizingMaskIntoConstraints = false
        view.colorPixelFormat = .bgra8Unorm
        view.clearColor = MTLClearColorMake(0, 0, 0, 0)
        view.layer?.isOpaque = false
        view.enableSetNeedsDisplay = false
        view.isPaused = false
        view.preferredFramesPerSecond = 60

        guard let renderer = MetalMeterRenderer(device: device, view: view) else {
            installFallbackLabel("Metal unavailable")
            return
        }

        view.delegate = renderer
        addSubview(view)
        NSLayoutConstraint.activate([
            view.leadingAnchor.constraint(equalTo: leadingAnchor),
            view.trailingAnchor.constraint(equalTo: trailingAnchor),
            view.topAnchor.constraint(equalTo: topAnchor),
            view.bottomAnchor.constraint(equalTo: bottomAnchor)
        ])

        self.metalView = view
        self.renderer = renderer
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }

    func update(levels: [MetalMeterLevel]) {
        renderer?.update(levels: levels)
        fallbackLabel?.stringValue = levels.isEmpty ? "No meter levels" : "Metal unavailable"
    }

    private func installFallbackLabel(_ text: String) {
        let label = NSTextField(labelWithString: text)
        label.alignment = .center
        label.textColor = .secondaryLabelColor
        label.translatesAutoresizingMaskIntoConstraints = false
        addSubview(label)
        NSLayoutConstraint.activate([
            label.leadingAnchor.constraint(equalTo: leadingAnchor),
            label.trailingAnchor.constraint(equalTo: trailingAnchor),
            label.centerYAnchor.constraint(equalTo: centerYAnchor)
        ])
        fallbackLabel = label
    }
}

private struct MetalMeterBar {
    var frame: SIMD4<Float>
    var data: SIMD4<Float>
}

private final class MetalMeterRenderer: NSObject, MTKViewDelegate {
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private let pipeline: MTLRenderPipelineState
    private let lock = NSLock()

    private var targets: [MetalMeterLevel] = []
    private var current: [Float] = []
    private var peaks: [Float] = []
    private var lastFrameTime = CACurrentMediaTime()

    init?(device: MTLDevice, view: MTKView) {
        self.device = device
        guard let commandQueue = device.makeCommandQueue(),
              let library = try? device.makeLibrary(source: Self.shaderSource, options: nil),
              let vertex = library.makeFunction(name: "meterVertex"),
              let fragment = library.makeFunction(name: "meterFragment") else {
            return nil
        }

        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = vertex
        descriptor.fragmentFunction = fragment
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = true
        descriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        descriptor.colorAttachments[0].sourceAlphaBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha

        guard let pipeline = try? device.makeRenderPipelineState(descriptor: descriptor) else {
            return nil
        }

        self.commandQueue = commandQueue
        self.pipeline = pipeline
        super.init()
    }

    func update(levels: [MetalMeterLevel]) {
        lock.lock()
        targets = levels
        if current.count != levels.count {
            current = levels.map { scaledLevel($0.normalized) }
            peaks = current
        }
        lock.unlock()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        let now = CACurrentMediaTime()
        let dt = min(max(Float(now - lastFrameTime), 0.001), 0.1)
        lastFrameTime = now

        lock.lock()
        let levels = targets
        if current.count != levels.count {
            current = levels.map { scaledLevel($0.normalized) }
            peaks = current
        }

        for index in levels.indices {
            let target = scaledLevel(levels[index].normalized)
            let timeConstant: Float = target > current[index] ? 0.035 : 0.52
            let blend = 1.0 - exp(-dt / timeConstant)
            current[index] += (target - current[index]) * blend

            if target > peaks[index] {
                peaks[index] = target
            } else {
                peaks[index] = max(current[index], peaks[index] - dt * 0.48)
            }
        }

        let bars = makeBars(levels: levels, current: current, peaks: peaks)
        lock.unlock()

        guard !bars.isEmpty,
              let drawable = view.currentDrawable,
              let renderPass = view.currentRenderPassDescriptor,
              let commandBuffer = commandQueue.makeCommandBuffer(),
              let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) else {
            return
        }

        let buffer = device.makeBuffer(bytes: bars,
                                       length: MemoryLayout<MetalMeterBar>.stride * bars.count,
                                       options: .storageModeShared)
        guard let buffer else {
            encoder.endEncoding()
            return
        }

        encoder.setRenderPipelineState(pipeline)
        encoder.setVertexBuffer(buffer, offset: 0, index: 0)
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 6, instanceCount: bars.count)
        encoder.endEncoding()
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private func makeBars(levels: [MetalMeterLevel],
                          current: [Float],
                          peaks: [Float]) -> [MetalMeterBar] {
        guard !levels.isEmpty else { return [] }

        let count = Float(levels.count)
        let spacing: Float = min(0.026, 0.22 / count)
        let totalSpacing = spacing * max(0, count - 1)
        let width = (1.84 - totalSpacing) / count
        var bars: [MetalMeterBar] = []
        bars.reserveCapacity(levels.count)

        for index in levels.indices {
            let x0 = -0.92 + Float(index) * (width + spacing)
            let x1 = x0 + width
            let level = min(max(current[index], 0), 1)
            let peak = min(max(peaks[index], 0), 1)
            bars.append(MetalMeterBar(frame: SIMD4<Float>(x0, -0.88, x1, 0.88),
                                      data: SIMD4<Float>(level, peak, levels[index].colorFamily, 0)))
        }
        return bars
    }

    private func scaledLevel(_ value: Float) -> Float {
        let clamped = min(max(value, 0), 1)
        return pow(clamped, 0.42)
    }

    private static let shaderSource = """
    #include <metal_stdlib>
    using namespace metal;

    struct Bar {
        float4 frame;
        float4 data;
    };

    struct VertexOut {
        float4 position [[position]];
        float2 uv;
        float level;
        float peak;
        float colorFamily;
    };

    vertex VertexOut meterVertex(uint vertexID [[vertex_id]],
                                 uint instanceID [[instance_id]],
                                 const device Bar *bars [[buffer(0)]]) {
        float2 unitPositions[6] = {
            float2(0.0, 0.0), float2(1.0, 0.0), float2(0.0, 1.0),
            float2(1.0, 0.0), float2(1.0, 1.0), float2(0.0, 1.0)
        };

        Bar bar = bars[instanceID];
        float2 uv = unitPositions[vertexID];
        float2 pos = mix(bar.frame.xy, bar.frame.zw, uv);

        VertexOut out;
        out.position = float4(pos, 0.0, 1.0);
        out.uv = uv;
        out.level = bar.data.x;
        out.peak = bar.data.y;
        out.colorFamily = bar.data.z;
        return out;
    }

    fragment float4 meterFragment(VertexOut in [[stage_in]]) {
        float3 familyA = float3(0.18, 0.48, 1.00);
        float3 familyB = float3(0.20, 0.78, 0.40);
        float3 familyC = float3(0.20, 0.74, 0.78);
        float3 familyD = float3(0.72, 0.42, 0.98);

        float family = in.colorFamily;
        float3 base = family < 0.5 ? familyA : (family < 1.5 ? familyB : (family < 2.5 ? familyC : familyD));

        float hot = smoothstep(0.78, 1.0, in.level);
        float warn = smoothstep(0.58, 0.90, in.level);
        float3 fill = mix(base, float3(1.0, 0.78, 0.18), warn);
        fill = mix(fill, float3(1.0, 0.20, 0.16), hot);

        float lit = step(in.uv.y, in.level);
        float peakLine = 1.0 - smoothstep(0.0, 0.018, abs(in.uv.y - in.peak));
        float segment = smoothstep(0.018, 0.026, fract(in.uv.y * 12.0));

        float3 dim = base * 0.24;
        float3 color = mix(dim, fill, lit);
        color *= mix(0.82, 1.0, segment);
        color = mix(color, float3(1.0), peakLine * 0.85);

        float alpha = mix(0.18, 0.96, lit);
        alpha = max(alpha, peakLine * 0.95);
        return float4(color, alpha);
    }
    """
}

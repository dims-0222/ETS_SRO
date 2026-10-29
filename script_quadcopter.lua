function sysCall_init()
    model = sim.getObject('..')
    laserHandle = sim.getObject("../sensor")
    joint0Handle = sim.getObject("../joint0")
    joint1Handle = sim.getObject("../joint1")

    red = {1, 0, 0}

    points = sim.addDrawingObject(
        sim.drawing_spherepts,
        0.01,
        0,
        -1,
        100000,
        nil,
        nil,
        nil,
        red
    )

    horizontalScanningAngle = 90 * math.pi / 180
    verticalScanningAngle = 90 * math.pi / 180

    -- Signal untuk komunikasi dengan Python
    scanSequence = 0

    sim.setInt32Signal('laserScanSeq', 0)
    sim.setStringSignal('laserPointCloud', '')

    print('[LiDAR] Scanner initialized')
    print('[LiDAR] Sensor handle: ' .. laserHandle)
end


function sysCall_sensing()
    -- UI untuk pengaturan sudut scan
    local s = sim.getObjectSel()
    local show = (s and #s == 1 and s[1] == model)

    if show then
        if not ui then
            local xml = [[
                <ui title="xxxx" closeable="false"
                    placement="relative" layout="form">
                    <label id="1" text=""/>
                    <hslider id="2" on-change="angleHMoved"/>
                    <label id="3" text=""/>
                    <hslider id="4" on-change="angleVMoved"/>
                </ui>
            ]]

            ui = simUI.create(xml)

            if uiPos then
                simUI.setPosition(ui, uiPos[1], uiPos[2])
            else
                uiPos = {}
                uiPos[1], uiPos[2] = simUI.getPosition(ui)
            end

            simUI.setTitle(ui, sim.getObjectAlias(model, 1))

            simUI.setLabelText(
                ui, 1,
                'Horizontal angle (' ..
                math.floor(horizontalScanningAngle * 180 / math.pi + 0.5) ..
                ' deg)'
            )

            simUI.setSliderValue(
                ui, 2,
                (horizontalScanningAngle * 180 / math.pi - 10) / 1.7
            )

            simUI.setLabelText(
                ui, 3,
                'Vertical angle (' ..
                math.floor(verticalScanningAngle * 180 / math.pi + 0.5) ..
                ' deg)'
            )

            simUI.setSliderValue(
                ui, 4,
                (verticalScanningAngle * 180 / math.pi - 10) / 1.7
            )
        end
    else
        if ui then
            uiPos[1], uiPos[2] = simUI.getPosition(ui)
            simUI.destroy(ui)
            ui = nil
        end
    end

    -- Bersihkan gambar scan sebelumnya
    sim.addDrawingObjectItem(points, nil)

    local scanPoints = {}

    local dv = verticalScanningAngle / 31
    local dh = horizontalScanningAngle / 31
    local pv = -verticalScanningAngle / 2

    -- Scan 32 x 32 = 1024 arah
    for y = 0, 31 do
        sim.setJointPosition(joint0Handle, pv)

        local ph = -horizontalScanningAngle / 2

        for x = 0, 31 do
            sim.setJointPosition(joint1Handle, ph)

            -- Matriks transformasi sensor ke koordinat dunia
            local m = sim.getObjectMatrix(laserHandle)

            -- Deteksi menggunakan mekanisme sensor asli
            local res, dist, pt =
                sim.handleProximitySensor(laserHandle)

            if res == 1 and pt then
                -- Transformasi titik lokal ke koordinat dunia
                local worldPoint = sim.multiplyVector(m, pt)

                -- Visualisasi titik merah di CoppeliaSim
                sim.addDrawingObjectItem(points, worldPoint)

                -- Simpan koordinat dunia sebagai CSV
                table.insert(
                    scanPoints,
                    string.format(
                        "%.6f,%.6f,%.6f",
                        worldPoint[1],
                        worldPoint[2],
                        worldPoint[3]
                    )
                )
            end

            ph = ph + dh
        end

        pv = pv + dv
    end

    -- Kirim seluruh titik hasil satu siklus scan
    sim.setStringSignal(
        'laserPointCloud',
        table.concat(scanPoints, '\n')
    )

    -- Naikkan nomor scan setelah data diterbitkan
    scanSequence = scanSequence + 1

    sim.setInt32Signal(
        'laserScanSeq',
        scanSequence
    )
end


function angleHMoved(ui, id, v)
    horizontalScanningAngle = math.pi * (10 + 1.7 * v) / 180

    simUI.setLabelText(
        ui, 1,
        'Horizontal angle (' ..
        math.floor(horizontalScanningAngle * 180 / math.pi + 0.5) ..
        ' deg)'
    )
end


function angleVMoved(ui, id, v)
    verticalScanningAngle = math.pi * (10 + 1.7 * v) / 180

    simUI.setLabelText(
        ui, 3,
        'Vertical angle (' ..
        math.floor(verticalScanningAngle * 180 / math.pi + 0.5) ..
        ' deg)'
    )
end


function sysCall_cleanup()
    if ui then
        simUI.destroy(ui)
        ui = nil
    end

    print('[LiDAR] Scanner stopped')
end

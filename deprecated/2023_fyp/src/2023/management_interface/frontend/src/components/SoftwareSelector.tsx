
import {Button, Form, Stack} from "react-bootstrap"
import {useEffect, useState} from "react";
import {backendURL} from "../App";
import {encode} from "base-64"
import {StringSelector} from "./StringSelector";
import {StringCard} from "./StringCard";

const SoftwareSelector = () => {

    const [dockerfilePath, setDockerfilePath] = useState("../../../../")
    const [dockerImageName, setDockerImageName] = useState("")

    const [remoteImages, setRemoteImages] = useState<string[]>([])
    const [selectedRemoteImages, setSelectedRemoteImages] = useState<string[]>([])

    const [localImages, setLocalImages] = useState<string[]>([])
    const [selectedLocalImages, setSelectedLocalImages] = useState<string[]>([])

    const [building, setBuilding] = useState(false)

    const getRegistryRepos = () => {
        fetch(backendURL + "getRemoteImages", {
            headers: {
                Authorization: `Basic ${encode('test:testpassword')}`
            }
        })
            .then(result => {
                result.json().then(json => setRemoteImages(json["repositories"]))
            }).catch(e => console.log(e))
    }

    useEffect(() => {
        getRegistryRepos()
        getLocalImages()
    }, [])

    const getLocalImages = () => {
        fetch(backendURL + "getLocalImages")
            .then(result => {
                result.json().then(json => setLocalImages(json))
            }).catch(e => console.log(e))
    }

    const downloadImage = (name: String) => {
        fetch(backendURL + "remoteToLocal", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                imageName: name
            })
        }).then(getLocalImages)
    }

    const makeBuildRequest = () => {
        setBuilding(true)
        fetch(backendURL + "build", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                path: dockerfilePath,
                name: dockerImageName
            })
        }).then(() => {
            setBuilding(false)
            getLocalImages()
        }).catch(() => setBuilding(false))
    }

    return (
        <div className="SoftwareSelector">

            <Stack direction="horizontal" gap={3}>
                <Stack direction="vertical">

                    <h2>Available Images</h2>

                    <StringSelector items={remoteImages} setSelectedItems={setSelectedRemoteImages} />

                    <Button
                        disabled={selectedRemoteImages.length === 0}
                        onClick={() => {
                            selectedRemoteImages.forEach(image => downloadImage(image))
                        }}
                    >
                        {">>>"}
                    </Button>

                    <Form.Label style={{textAlign: "left"}}>Build an image from a dockerfile:</Form.Label>
                    <Form.Control value={dockerfilePath} onChange={event => setDockerfilePath(event.target.value)}/>

                    <Form.Label style={{textAlign: "left"}}>Image Name:</Form.Label>
                    <Form.Control value={dockerImageName} onChange={event => setDockerImageName(event.target.value)}/>
                    <Button className="mt-2" disabled={building} onClick={makeBuildRequest}>{!building ? "Build" : "Building..."}</Button>
                </Stack>
                <div className="vr" />
                <Stack direction="vertical">
                    <h2>Local Images</h2>
                    <StringSelector items={localImages} setSelectedItems={setSelectedLocalImages}/>
                </Stack>
            </Stack>



        </div>
    )
}

export {SoftwareSelector}
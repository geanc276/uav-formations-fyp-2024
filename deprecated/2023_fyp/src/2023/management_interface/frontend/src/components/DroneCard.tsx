import {Card} from "react-bootstrap";
const DroneCard = ({onClick, selected, drone}: DroneCardProps) => {

    const {name, ip} = drone

    return (
        <Card
            className="DroneCard "
            bg={selected ? "primary": "secondary"}
            text="light"
            onClick={onClick}

        >
            <Card.Body>
                <Card.Title>{name}</Card.Title>
                <Card.Text>{ip}</Card.Text>
            </Card.Body>

        </Card>
    )
}

export {DroneCard}
export type DroneCardProps = {onClick: () => void, selected:boolean, drone:Drone}
export type Drone = {name: string, ip:string}
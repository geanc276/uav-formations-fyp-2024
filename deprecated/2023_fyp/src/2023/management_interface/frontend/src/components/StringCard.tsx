import {Card} from "react-bootstrap";
const StringCard = ({onClick, selected, text}: StringCardProps) => {

    return (
        <Card
            className="DroneCard "
            bg={selected ? "primary": "secondary"}
            text="light"
            onClick={onClick}

        >
            <Card.Body>
                <Card.Text>{text}</Card.Text>
            </Card.Body>

        </Card>
    )
}

export {StringCard}
export type StringCardProps = {onClick: () => void, selected:boolean, text: string}
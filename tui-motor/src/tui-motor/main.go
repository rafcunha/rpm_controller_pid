package main

import (
	"fmt"
	"log"
	"os"
	"strconv"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/tarm/serial"
)

type model struct {
	items         []string
	cursor        int
	selectedIndex int
	state         string
	serial        *serial.Port
	velocityInput string
}

func initialModel(porta *serial.Port) model {
	return model{
		items:         []string{"Parar Motor", "Escolher Velocidade", "Modo de Teste"},
		cursor:        0,
		selectedIndex: -1,
		state:         "list",
		serial:        porta,
		velocityInput: "",
	}
}

func (m model) Init() tea.Cmd {
	return nil
}

func (m model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	switch msg := msg.(type) {
	case tea.KeyMsg:
		switch m.state {
		case "list":
			switch msg.String() {
			case "up", "k":
				if m.cursor > 0 {
					m.cursor--
				}
			case "down", "j":
				if m.cursor < len(m.items)-1 {
					m.cursor++
				}
			case "enter":
				m.selectedIndex = m.cursor

				switch m.items[m.selectedIndex] {
				case "Modo de Teste":
					m.state = "view"
					_, err := m.serial.Write([]byte("t"))
					if err != nil {
						log.Println("Serial write error:", err)
					}
				case "Parar Motor":
					m.state = "view"
					_, err := m.serial.Write([]byte("d"))
					if err != nil {
						log.Println("Serial write error:", err)
					}
				case "Escolher Velocidade":
					m.state = "velocityInput"
					m.velocityInput = ""
				}

			case "ctrl+c", "q":
				return m, tea.Quit
			}

		case "velocityInput":
			switch msg.String() {
			case "1", "2", "3":
				m.velocityInput = msg.String()
			case "enter":
				if m.velocityInput != "" {
					v, err := strconv.Atoi(m.velocityInput)
					if err == nil && v >= 1 && v <= 3 {
						_, err := m.serial.Write([]byte("l" + m.velocityInput))
						if err != nil {
							log.Println("Serial write error:", err)
						}
						m.state = "view"
					}
				}
			case "esc", "q":
				m.state = "list"
			}

		case "view":
			switch msg.String() {
			case "esc", "q":
				m.state = "list"
			case "ctrl+c":
				return m, tea.Quit
			}
		}
	}
	return m, nil
}

func (m model) View() string {
	switch m.state {
	case "list":
		s := "Selecione uma das opcoes abaixo:\n\n"
		for i, item := range m.items {
			cursor := " "
			if m.cursor == i {
				cursor = ">"
			}

			s += fmt.Sprintf("%s %s\n", cursor, item)
		}
		s += "\nUtilize ↑/↓ para se mover, enter para selecionar, q para sair."
		return s

	case "velocityInput":
		return fmt.Sprintf(
			"Escolha a velocidade (1 - 3): %s\n\nPressione enter para confirmar, esc ou q para voltar.",
			m.velocityInput,
		)

	case "view":
		selected := "None"
		if m.selectedIndex >= 0 && m.selectedIndex < len(m.items) {
			selected = m.items[m.selectedIndex]
			if selected == "Escolher Velocidade" && m.velocityInput != "" {
				selected = fmt.Sprintf("%s (velocidade %s)", selected, m.velocityInput)
			}
		}
		return fmt.Sprintf(
			"Item Selecionado: %s\n\nPressione esc ou q para voltar.",
			selected,
		)

	default:
		return "Unknown state"
	}
}

func main() {
	c := &serial.Config{
		Name: "/dev/ttyACM1",
		Baud: 115200,
	}

	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}
	defer s.Close()

	p := tea.NewProgram(initialModel(s))
	if err := p.Start(); err != nil {
		fmt.Println("Error running program:", err)
		os.Exit(1)
	}
}


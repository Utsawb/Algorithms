#include <array>
#include <iostream>

#include "algorithms.hh"

enum class CellState : uint8_t
{
    EMPTY,
    X,
    O
};

struct GameState
{
    std::array<std::array<CellState, 3>, 3> board;
    CellState turn;
};

bool has_won(const GameState &game_state, CellState player)
{
    const auto &board = game_state.board;

    // Check rows
    for (int i = 0; i < 3; ++i)
    {
        if (board[i][0] == player && board[i][1] == player && board[i][2] == player)
            return true;
    }

    // Check columns
    for (int i = 0; i < 3; ++i)
    {
        if (board[0][i] == player && board[1][i] == player && board[2][i] == player)
            return true;
    }

    // Check diagonals
    if (board[0][0] == player && board[1][1] == player && board[2][2] == player)
        return true;
    if (board[0][2] == player && board[1][1] == player && board[2][0] == player)
        return true;

    return false;
}

bool is_terminal(const GameState &game_state)
{
    // Check for a win for either player
    if (has_won(game_state, CellState::X) || has_won(game_state, CellState::O))
    {
        return true;
    }

    // Check for a draw (full board)
    // If we find even one empty cell, the game is not over (unless someone won)
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            if (game_state.board[r][c] == CellState::EMPTY)
            {
                return false; // Found an empty spot, game is not terminal
            }
        }
    }

    // If no wins and no empty spots, it's a draw (which is a terminal state)
    return true;
}

float heuristic(const GameState &game_state)
{
    // This heuristic is only designed for terminal nodes as requested.

    if (has_won(game_state, CellState::X))
    {
        // 'X' (Maximizer) wins
        return 1.0f;
    }
    else if (has_won(game_state, CellState::O))
    {
        // 'O' (Minimizer) wins
        return -1.0f;
    }
    else
    {
        return 0.0f;
    }
}

std::vector<GameState> subsequent(const GameState &game_state)
{
    std::vector<GameState> next_states;

    // If the current state is already terminal, there are no subsequent states
    if (is_terminal(game_state))
    {
        return next_states;
    }

    // Determine whose turn it will be in the *next* state
    CellState next_player_turn = (game_state.turn == CellState::X) ? CellState::O : CellState::X;

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            // Check for an empty spot
            if (game_state.board[r][c] == CellState::EMPTY)
            {
                // This is a valid move. Create a new state for it.
                GameState new_state;
                new_state.board = game_state.board;      // 1. Copy the current board
                new_state.board[r][c] = game_state.turn; // 2. Place the current player's mark
                new_state.turn = next_player_turn;       // 3. Set the turn for the next player

                next_states.push_back(new_state);
            }
        }
    }

    return next_states;
}

void init_game(GameState &state)
{
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            state.board[r][c] = CellState::EMPTY;
        }
    }
    state.turn = CellState::X; // X always starts
}

char cell_to_char(CellState cell)
{
    switch (cell)
    {
    case CellState::X:
        return 'X';
    case CellState::O:
        return 'O';
    default:
        return ' ';
    }
}

void print_board(const GameState &state)
{
    // Simple clear screen for better readability
    std::cout << "\033[2J\033[1;1H";
    std::cout << "  0 1 2\n";
    for (int r = 0; r < 3; ++r)
    {
        std::cout << r << " ";
        for (int c = 0; c < 3; ++c)
        {
            std::cout << cell_to_char(state.board[r][c]);
            if (c < 2)
                std::cout << "|";
        }
        std::cout << "\n";
        if (r < 2)
            std::cout << "  -+-+-\n";
    }
    std::cout << "\n";
}

void get_player_input(GameState &state)
{
    int row = -1, col = -1;
    std::cout << "Player " << cell_to_char(state.turn) << ", enter your move (row col): ";

    if (!(std::cin >> row >> col))
    {
        std::cout << "Invalid input. Please enter two numbers.\n";
        // Clear the error state and discard the bad input
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        throw;
    }

    // Check bounds
    if (row < 0 || row > 2 || col < 0 || col > 2)
    {
        std::cout << "Invalid move. Row and column must be 0, 1, or 2.\n";
        throw;
    }

    // Check if cell is empty
    if (state.board[row][col] != CellState::EMPTY)
    {
        std::cout << "Invalid move. That cell is already taken.\n";
        throw;
    }

    // If all checks pass, make the move
    state.board[row][col] = state.turn;
    state.turn = (state.turn == CellState::X) ? CellState::O : CellState::X;
}

void computer_move(GameState &game_state)
{
    std::vector<GameState> moves = subsequent(game_state);

    std::size_t max_index = 0;
    float max_score = jj::alphabeta<float, GameState, std::size_t, is_terminal, heuristic, subsequent>(
        moves.at(0), std::numeric_limits<std::size_t>::max() / 2, std::numeric_limits<float>::min() / 2,
        std::numeric_limits<float>::max() / 2, true);

    for (std::size_t i = 1; i < moves.size(); ++i)
    {
        float current_score = jj::alphabeta<float, GameState, std::size_t, is_terminal, heuristic, subsequent>(
            moves.at(i), std::numeric_limits<std::size_t>::max() / 2, std::numeric_limits<float>::min() / 2,
            std::numeric_limits<float>::max() / 2, true);

        if (current_score > max_score)
        {
            max_score = current_score;
            max_index = i;
        }
    }

    game_state = moves.at(max_index);
}

int main(void)
{
    GameState game_state;
    init_game(game_state);

    while (true)
    {
        print_board(game_state);

        if (game_state.turn == CellState::X)
        {
            computer_move(game_state);
        }
        else if (game_state.turn == CellState::O)
        {
            get_player_input(game_state);
        }

        if (is_terminal(game_state))
        {
            break;
        }
    }

    print_board(game_state);

    return 0;
}

#pragma once

#include <QVector>

// Snapshot-based undo for the graphical section panels. Each mutating
// operation pushes the complete before/after state it touched (section
// text, spline JSON, regenerated files — all small), so undo restores an
// absolute state and cannot conflict with the text editor's own stack:
// whatever the current state is, undo simply reinstates the snapshot.
template <typename State> class PanelUndoStack
{
public:
    void push(State before, State after)
    {
        entries_.resize(index_); // drop the redo tail
        entries_.append({std::move(before), std::move(after)});
        if (entries_.size() > kDepthLimit)
            entries_.removeFirst();
        index_ = entries_.size();
    }

    bool canUndo() const { return index_ > 0; }
    bool canRedo() const { return index_ < entries_.size(); }

    // The state to reinstate, or nullptr when the stack end is reached.
    const State *undo()
    {
        if (!canUndo())
            return nullptr;
        --index_;
        return &entries_.at(index_).before;
    }

    const State *redo()
    {
        if (!canRedo())
            return nullptr;
        const State *state = &entries_.at(index_).after;
        ++index_;
        return state;
    }

private:
    struct Entry
    {
        State before;
        State after;
    };
    static constexpr qsizetype kDepthLimit = 50;

    QVector<Entry> entries_;
    qsizetype index_ = 0;
};

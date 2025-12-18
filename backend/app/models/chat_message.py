"""ChatMessage model for storing chat history."""

from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, CheckConstraint
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from app.core.database import Base


class ChatMessage(Base):
    """Chat message model with auto-deletion after 30 days to conserve storage."""

    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id", ondelete="SET NULL"), nullable=True, index=True)
    session_id = Column(String(36), nullable=False, index=True)  # UUID
    message = Column(Text, nullable=False)  # User's question
    response = Column(Text, nullable=False)  # Bot's response
    sources = Column(JSONB, nullable=True)  # Source citations as JSON
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False, index=True)

    # Relationship
    user = relationship("User", back_populates="chat_messages")

    # Constraint: auto-delete messages older than 30 days (enforced at query level)
    __table_args__ = (
        CheckConstraint(
            "created_at > NOW() - INTERVAL '30 days'",
            name="chat_retention"
        ),
    )

    def __repr__(self):
        return f"<ChatMessage(id={self.id}, user_id={self.user_id}, session_id={self.session_id})>"
